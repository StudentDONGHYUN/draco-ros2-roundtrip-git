#!/usr/bin/env python3
"""
Streaming client for live ROS2 LiDAR topics.

- Subscribe to a PointCloud2 topic
- Encode each frame to Draco (.drc) and send to a remote server
- Receive decoded PLY bytes, compute metrics, publish both source/decoded clouds
- Log bandwidth usage
"""

from __future__ import annotations

import argparse
import io
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

try:
    from plyfile import PlyData, PlyElement
except ImportError as exc:  # pragma: no cover - required dependency
    raise SystemExit("plyfile module is required: pip install plyfile") from exc

try:
    from scipy.spatial import cKDTree  # type: ignore
    _HAVE_SCIPY = True
except Exception:  # pragma: no cover
    _HAVE_SCIPY = False


def find_exe(name: str, hint: Optional[str] = None) -> Path:
    if hint:
        p = Path(hint).expanduser().resolve()
        if p.is_file():
            return p
        candidate = p / name
        if candidate.exists():
            return candidate
    from shutil import which
    found = which(name)
    if not found:
        raise FileNotFoundError(f"Unable to locate {name}")
    return Path(found)


def send_message(sock: socket.socket, name: str, payload: bytes) -> None:
    name_b = name.encode('utf-8')
    sock.sendall(struct.pack('!I', len(name_b)))
    sock.sendall(name_b)
    sock.sendall(struct.pack('!Q', len(payload)))
    sock.sendall(payload)


def recv_message(sock: socket.socket) -> Optional[tuple[str, bytes]]:
    header = sock.recv(4)
    if not header:
        return None
    name_len = struct.unpack('!I', header)[0]
    if name_len == 0:
        return None
    name = sock.recv(name_len).decode('utf-8')
    size = struct.unpack('!Q', sock.recv(8))[0]
    payload = bytearray()
    remaining = size
    while remaining:
        chunk = sock.recv(min(65536, remaining))
        if not chunk:
            raise ConnectionError("connection closed prematurely")
        payload.extend(chunk)
        remaining -= len(chunk)
    return name, bytes(payload)


def to_xyz_array_from_pc2(msg: PointCloud2) -> np.ndarray:
    try:
        arr = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape((-1, 3))
        elif arr.shape[1] > 3:
            arr = arr[:, :3]
        return arr
    except Exception:
        pts = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            try:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
            except Exception:
                x = float(p['x'])
                y = float(p['y'])
                z = float(p['z'])
            pts.append((x, y, z))
        return np.asarray(pts, dtype=np.float32)


def write_ply(path: Path, pts: np.ndarray) -> None:
    verts = np.zeros(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
    if len(pts):
        verts['x'] = pts[:, 0]
        verts['y'] = pts[:, 1]
        verts['z'] = pts[:, 2]
    PlyData([PlyElement.describe(verts, 'vertex')], text=False).write(path)


def encode_ply(encoder: Path, pts: np.ndarray, tmp_dir: Path,
               stem: str, cl: int, qp: int, qg: int, extra: list[str]) -> Tuple[bytes, Path]:
    tmp_dir.mkdir(parents=True, exist_ok=True)
    ply_path = tmp_dir / f"{stem}.ply"
    drc_path = tmp_dir / f"{stem}.drc"
    write_ply(ply_path, pts)
    cmd = [str(encoder), '-i', str(ply_path), '-o', str(drc_path),
           '-cl', str(cl), '-qp', str(qp), '-qg', str(qg)]
    if extra:
        cmd.extend(extra)
    rc = subprocess.run(cmd, capture_output=True, text=True)
    if rc.returncode != 0:
        raise RuntimeError(f"draco_encoder failed: {rc.stderr.strip()}\n{rc.stdout.strip()}")
    return drc_path.read_bytes(), ply_path


def load_points_from_ply_bytes(data: bytes) -> np.ndarray:
    ply = PlyData.read(io.BytesIO(data))
    verts = ply['vertex']
    arr = np.vstack((verts['x'], verts['y'], verts['z']))
    return arr.T.astype(np.float32)


def compute_metrics(src_pts: np.ndarray, dec_pts: np.ndarray, sample: int) -> Tuple[int, int, int, np.ndarray, float, np.ndarray, str, str]:
    n_src = len(src_pts)
    n_dec = len(dec_pts)
    diff = n_dec - n_src

    centroid_src = src_pts.mean(axis=0) if n_src else np.zeros(3, dtype=np.float32)
    centroid_dec = dec_pts.mean(axis=0) if n_dec else np.zeros(3, dtype=np.float32)
    centroid_delta = centroid_dec - centroid_src
    centroid_norm = float(np.linalg.norm(centroid_delta))

    bbox_src_min = src_pts.min(axis=0) if n_src else np.zeros(3, dtype=np.float32)
    bbox_src_max = src_pts.max(axis=0) if n_src else np.zeros(3, dtype=np.float32)
    bbox_dec_min = dec_pts.min(axis=0) if n_dec else np.zeros(3, dtype=np.float32)
    bbox_dec_max = dec_pts.max(axis=0) if n_dec else np.zeros(3, dtype=np.float32)
    bbox_delta = (bbox_dec_max - bbox_dec_min) - (bbox_src_max - bbox_src_min)

    mean_str = "n/a"
    max_str = "n/a"
    if n_src and n_dec:
        m = min(n_src, n_dec)
        if sample > 0:
            step = max(m // sample, 1)
            idx = np.arange(0, m, step)
        else:
            idx = np.arange(m)
        src_sample = src_pts[idx]
        dec_sample = dec_pts[idx]
        if _HAVE_SCIPY:
            kd_dec = cKDTree(dec_sample)
            kd_src = cKDTree(src_sample)
            d_src = kd_dec.query(src_sample, k=1)[0]
            d_dec = kd_src.query(dec_sample, k=1)[0]
            dists = np.concatenate([d_src, d_dec])
        else:
            length = min(len(src_sample), len(dec_sample))
            dists = np.linalg.norm(src_sample[:length] - dec_sample[:length], axis=1)
        if len(dists):
            mean_str = f"{float(np.mean(dists)):.3f}"
            max_str = f"{float(np.max(dists)):.3f}"
    return n_src, n_dec, diff, centroid_delta, centroid_norm, bbox_delta, mean_str, max_str


class StreamClientNode(Node):
    def __init__(self, name: str, topic: str, queue_obj: queue.Queue):
        super().__init__(name)
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(PointCloud2, topic, self._callback, qos)
        self.queue = queue_obj
        self.frame_index = 0

    def _callback(self, msg: PointCloud2) -> None:
        pts = to_xyz_array_from_pc2(msg)
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frame_index += 1
        self.queue.put((self.frame_index, stamp_sec, pts))


class PublisherNode(Node):
    def __init__(self, frame_id: str, topic_prefix: str, hz: float):
        super().__init__('stream_live_publisher')
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_src = self.create_publisher(PointCloud2, f"/{topic_prefix}/source", qos)
        self.pub_dec = self.create_publisher(PointCloud2, f"/{topic_prefix}/decoded", qos)
        self.frame_id = frame_id
        self.period = 1.0 / hz if hz > 0 else 0.01

    def publish_pair(self, pts_src: np.ndarray, pts_dec: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        msg_src = pc2.create_cloud_xyz32(header, pts_src.astype(np.float32))
        msg_dec = pc2.create_cloud_xyz32(header, pts_dec.astype(np.float32))
        self.pub_src.publish(msg_src)
        self.pub_dec.publish(msg_dec)


def worker_loop(queue_obj: queue.Queue, encoder: Path, tmp_dir: Path,
                cl: int, qp: int, qg: int, extra: list[str],
                server_addr: Tuple[str, int], publisher: PublisherNode,
                decoded_dir: Path, sample: int, stop_event: threading.Event) -> None:
    bytes_sent = 0
    bytes_received = 0
    start_time = time.monotonic()

    with socket.create_connection(server_addr) as sock:
        print(f"[CLIENT] Connected to {server_addr[0]}:{server_addr[1]}")
        try:
            while not stop_event.is_set():
                try:
                    frame = queue_obj.get(timeout=0.1)
                except queue.Empty:
                    continue
                frame_idx, stamp_sec, pts = frame
                stem = f"frame_{frame_idx:010d}"
                try:
                    drc_bytes, ply_path = encode_ply(encoder, pts, tmp_dir, stem, cl, qp, qg, extra)
                except Exception as exc:
                    print(f"[CLIENT] ENCODE FAIL {stem}: {exc}")
                    continue
                send_message(sock, f"{stem}.drc", drc_bytes)
                bytes_sent += len(drc_bytes)
                reply = recv_message(sock)
                if reply is None:
                    raise ConnectionError("Server closed connection")
                name, payload = reply
                if name.startswith('decode-error:'):
                    print(f"[CLIENT] Server decode error: {name}")
                    continue
                bytes_received += len(payload)
                out_path = decoded_dir / name
                out_path.write_bytes(payload)
                pts_dec = load_points_from_ply_bytes(payload)
                n_src, n_dec, diff, centroid_delta, centroid_norm, bbox_delta, mean_str, max_str = compute_metrics(pts, pts_dec, sample)
                print(f"[FRAME {frame_idx:04d}] stamp={stamp_sec:.3f}")
                print(f"  points:  src={n_src}  dec={n_dec}  diff={diff:+d}")
                print(f"  centroid diff (m):  x={centroid_delta[0]:+.3f}  y={centroid_delta[1]:+.3f}  z={centroid_delta[2]:+.3f}  ‖Δ‖={centroid_norm:.3f}")
                print(f"  bbox size Δ (m):   x={bbox_delta[0]:+.3f}  y={bbox_delta[1]:+.3f}  z={bbox_delta[2]:+.3f}")
                print(f"  mean|max dist (m): {mean_str}/{max_str}")
                publisher.publish_pair(pts, pts_dec)
        finally:
            send_message(sock, '', b'')

    elapsed = max(time.monotonic() - start_time, 1e-6)
    total = bytes_sent + bytes_received
    print("[CLIENT] ---- Bandwidth summary ----")
    print(f"  elapsed: {elapsed:.2f} s")
    print(f"  sent: {bytes_sent} bytes ({bytes_sent * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  received: {bytes_received} bytes ({bytes_received * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  total: {total} bytes ({total * 8 / elapsed / 1e6:.3f} Mbps)")


def main(argv: Iterable[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Live streaming client with replay")
    ap.add_argument('--topic', required=True, help="PointCloud2 topic to subscribe")
    ap.add_argument('--prefix', default='lidar', help="Frame name prefix")
    ap.add_argument('--encoder', default=None, help="Path to draco_encoder")
    ap.add_argument('--cl', type=int, default=8)
    ap.add_argument('--qp', type=int, default=12)
    ap.add_argument('--qg', type=int, default=10)
    ap.add_argument('--encoder-extra', nargs='*', default=[])
    ap.add_argument('--tmp-dir', default='data/live_tmp')
    ap.add_argument('--decoded-dir', default='data/decoded_from_server')
    ap.add_argument('--server-host', default='127.0.0.1')
    ap.add_argument('--server-port', type=int, default=5000)
    ap.add_argument('--play-frame-id', default='lidar_link')
    ap.add_argument('--play-topic-prefix', default='stream_pair')
    ap.add_argument('--play-hz', type=float, default=10.0)
    ap.add_argument('--play-sample', type=int, default=50000)
    ap.add_argument('--queue-size', type=int, default=10)
    args = ap.parse_args(argv)

    encoder = find_exe('draco_encoder', args.encoder)
    rclpy.init()
    tmp_dir = Path(args.tmp_dir).resolve()
    decoded_dir = Path(args.decoded_dir).resolve()
    tmp_dir.mkdir(parents=True, exist_ok=True)
    decoded_dir.mkdir(parents=True, exist_ok=True)

    frame_queue: queue.Queue = queue.Queue(maxsize=max(args.queue_size, 1))
    subscriber_node = StreamClientNode('stream_client_node', args.topic, frame_queue)
    publisher_node = PublisherNode(args.play_frame_id, args.play_topic_prefix, args.play_hz)

    stop_event = threading.Event()

    worker = threading.Thread(
        target=worker_loop,
        args=(frame_queue, encoder, tmp_dir, args.cl, args.qp, args.qg,
              args.encoder_extra, (args.server_host, args.server_port),
              publisher_node, decoded_dir, args.play_sample, stop_event),
        daemon=True
    )
    worker.start()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(subscriber_node)
        executor.add_node(publisher_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        worker.join()
        subscriber_node.destroy_node()
        publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
