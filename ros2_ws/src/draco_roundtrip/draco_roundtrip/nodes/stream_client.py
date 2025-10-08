#!/usr/bin/env python3
"""Stream rosbag frames, encode/send to server, replay decoded results to RViz."""

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
from typing import Iterable, Optional, Set, Tuple

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - during setup or if ament not available
    get_package_share_directory = None  # type: ignore

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

try:
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:  # pragma: no cover - optional dep
    _HAVE_O3D = False

try:
    from plyfile import PlyData
except ImportError as exc:
    raise SystemExit("plyfile module is required") from exc

try:
    from scipy.spatial import cKDTree  # type: ignore
    _HAVE_SCIPY = True
except Exception:
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


def resolve_qos_override() -> Optional[Path]:
    candidates: list[Path] = []
    if get_package_share_directory:
        try:
            share_dir = Path(get_package_share_directory('draco_roundtrip'))
            candidates.append(share_dir / 'config' / 'qos_override.yaml')
            candidates.append(share_dir / 'configs' / 'qos_override.yaml')
        except Exception:
            pass
    # Fall back to source tree locations (symlink install, dev runs)
    candidates.append(Path(__file__).resolve().parents[2] / 'configs' / 'qos_override.yaml')
    candidates.append(Path(__file__).resolve().parents[3] / 'configs' / 'qos_override.yaml')
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


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


def encode_ply(encoder: Path, ply_path: Path, out_dir: Path,
               cl: int, qp: int, qg: int, extra: list[str]) -> bytes:
    out_dir.mkdir(parents=True, exist_ok=True)
    drc_path = out_dir / (ply_path.stem + '.drc')
    cmd = [str(encoder), '-i', str(ply_path), '-o', str(drc_path),
           '-cl', str(cl), '-qp', str(qp), '-qg', str(qg)]
    if extra:
        cmd.extend(extra)
    rc = subprocess.run(cmd, capture_output=True, text=True)
    if rc.returncode != 0:
        raise RuntimeError(f"draco_encoder failed: {rc.stderr.strip()}\n{rc.stdout.strip()}")
    return drc_path.read_bytes()


def load_points_from_path(path: Path) -> np.ndarray:
    if _HAVE_O3D:
        pcd = o3d.io.read_point_cloud(str(path))
        return np.asarray(pcd.points, dtype=np.float32)
    ply = PlyData.read(str(path))
    verts = ply["vertex"]
    arr = np.vstack((verts["x"], verts["y"], verts["z"]))
    return arr.T.astype(np.float32)


def load_points_from_bytes(data: bytes) -> np.ndarray:
    ply = PlyData.read(io.BytesIO(data))
    verts = ply["vertex"]
    arr = np.vstack((verts["x"], verts["y"], verts["z"]))
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


def launch_bag_to_ply(root: Path, args: argparse.Namespace) -> subprocess.Popen:
    script = root / 'data' / 'bag_to_ply.py'
    cmd = [sys.executable, str(script),
           '--topic', args.topic,
           '--out', str(Path(args.ply_dir).resolve()),
           '--prefix', args.prefix,
           '--idle-timeout-sec', str(args.idle_timeout),
    ]
    if args.best_effort:
        cmd.append('--best-effort')
    if args.max_frames:
        cmd += ['--max-frames', str(args.max_frames)]
    return subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)


class PlaybackNode(Node):
    def __init__(self, queue_obj: queue.Queue, frame_id: str, topic_prefix: str, hz: float):
        super().__init__('stream_playback')
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_src = self.create_publisher(PointCloud2, f"/{topic_prefix}/source", qos)
        self.pub_dec = self.create_publisher(PointCloud2, f"/{topic_prefix}/decoded", qos)
        self.queue = queue_obj
        self.frame_id = frame_id
        self.period = 1.0 / hz if hz > 0 else 0.01
        self.timer = self.create_timer(self.period, self._tick)

    def _tick(self) -> None:
        try:
            item = self.queue.get_nowait()
        except queue.Empty:
            return
        if item is None:
            self.get_logger().info("Playback finished")
            rclpy.shutdown()
            return
        frame_idx, name, pts_src, pts_dec = item
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg_src = pc2.create_cloud_xyz32(header, pts_src)
        msg_dec = pc2.create_cloud_xyz32(header, pts_dec)
        self.pub_src.publish(msg_src)
        self.pub_dec.publish(msg_dec)
        self.get_logger().debug(f"Published frame {frame_idx}: {name}")


def start_playback_thread(queue_obj: queue.Queue, frame_id: str, topic_prefix: str, hz: float) -> threading.Thread:
    def _run():
        rclpy.init()
        node = PlaybackNode(queue_obj, frame_id, topic_prefix, hz)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


def main(argv: Iterable[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Streaming client with live playback")
    ap.add_argument('--bag', required=True)
    ap.add_argument('--topic', required=True)
    ap.add_argument('--prefix', required=True)
    ap.add_argument('--ply-dir', default='data/ply_stream')
    ap.add_argument('--encoder', default=None)
    ap.add_argument('--cl', type=int, default=8)
    ap.add_argument('--qp', type=int, default=12)
    ap.add_argument('--qg', type=int, default=10)
    ap.add_argument('--encoder-extra', nargs='*', default=[])
    ap.add_argument('--idle-timeout', type=float, default=10.0)
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--best-effort', action='store_true')
    ap.add_argument('--work-dir', default='data/client_tmp')
    ap.add_argument('--decoded-dir', default='data/decoded_from_server')
    ap.add_argument('--server-host', default='127.0.0.1')
    ap.add_argument('--server-port', type=int, default=5000)
    ap.add_argument('--play-frame-id', default='lidar_link')
    ap.add_argument('--play-topic-prefix', default='stream_pair')
    ap.add_argument('--play-hz', type=float, default=10.0)
    ap.add_argument('--play-sample', type=int, default=50000)
    args = ap.parse_args(argv)

    encoder = find_exe('draco_encoder', args.encoder)
    root = Path(__file__).resolve().parents[1]

    ply_dir = Path(args.ply_dir).resolve()
    ply_dir.mkdir(parents=True, exist_ok=True)
    work_dir = Path(args.work_dir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    decoded_dir = Path(args.decoded_dir).resolve()
    decoded_dir.mkdir(parents=True, exist_ok=True)

    bag_cmd = ['ros2', 'bag', 'play', str(Path(args.bag).resolve())]
    qos_override = resolve_qos_override()
    if qos_override is not None:
        bag_cmd += ['--qos-profile-overrides-path', str(qos_override)]
    else:
        print('[CLIENT] WARN: QoS override file not found, falling back to recorded QoS', file=sys.stderr)
    bag_process = subprocess.Popen(bag_cmd)
    saver_proc = launch_bag_to_ply(root, args)

    to_play: queue.Queue = queue.Queue()
    playback_thread = start_playback_thread(to_play, args.play_frame_id, args.play_topic_prefix, args.play_hz)

    processed: Set[Path] = set()
    frame_idx = 0
    start_time = time.monotonic()
    bytes_sent = 0
    bytes_received = 0

    try:
        with socket.create_connection((args.server_host, args.server_port)) as sock:
            print(f"[CLIENT] Connected to {args.server_host}:{args.server_port}")
            try:
                while True:
                    new_files = sorted(ply_dir.glob(f"{args.prefix}_*.ply"))
                    for ply_path in new_files:
                        if ply_path in processed:
                            continue
                        try:
                            drc_bytes = encode_ply(encoder, ply_path, work_dir,
                                                   args.cl, args.qp, args.qg,
                                                   args.encoder_extra)
                        except Exception as exc:
                            print(f"[CLIENT] ENCODE FAIL {ply_path.name}: {exc}")
                            processed.add(ply_path)
                            continue
                        send_message(sock, ply_path.name.replace('.ply', '.drc'), drc_bytes)
                        bytes_sent += len(drc_bytes)
                        print(f"[CLIENT] Sent {ply_path.name} ({len(drc_bytes)} bytes)")
                        reply = recv_message(sock)
                        if reply is None:
                            raise ConnectionError("Server closed connection")
                        name, payload = reply
                        if name.startswith('decode-error:'):
                            print(f"[CLIENT] Server decode error: {name}")
                        else:
                            out_path = decoded_dir / name
                            out_path.write_bytes(payload)
                            pts_src = load_points_from_path(ply_path)
                            pts_dec = load_points_from_bytes(payload)
                            frame_idx += 1
                            n_src, n_dec, diff, centroid_delta, centroid_norm, bbox_delta, mean_str, max_str = compute_metrics(
                                pts_src, pts_dec, args.play_sample)
                            print(f"[FRAME {frame_idx:04d}] {ply_path.stem}")
                            print(f"  points:  src={n_src}  dec={n_dec}  diff={diff:+d}")
                            print(
                                "  centroid diff (m):  "
                                f"x={centroid_delta[0]:+.3f}  y={centroid_delta[1]:+.3f}  z={centroid_delta[2]:+.3f}  ‖Δ‖={centroid_norm:.3f}"
                            )
                            print(
                                "  bbox size Δ (m):   "
                                f"x={bbox_delta[0]:+.3f}  y={bbox_delta[1]:+.3f}  z={bbox_delta[2]:+.3f}"
                            )
                            print(f"  mean|max dist (m): {mean_str}/{max_str}")
                            to_play.put((frame_idx, ply_path.stem, pts_src.astype(np.float32), pts_dec.astype(np.float32)))
                            bytes_received += len(payload)
                            print(f"[CLIENT] Received {name} ({len(payload)} bytes)")
                        processed.add(ply_path)
                    if saver_proc.poll() is not None and bag_process.poll() is not None:
                        if len(processed) == len(list(ply_dir.glob(f"{args.prefix}_*.ply"))):
                            break
                    time.sleep(0.2)
            finally:
                send_message(sock, '', b'')
    finally:
        saver_proc.wait()
        bag_process.wait()
        to_play.put(None)
        playback_thread.join(timeout=5)
        print("[CLIENT] Streaming playback completed")
        elapsed = max(time.monotonic() - start_time, 1e-6)
        total = bytes_sent + bytes_received
        print("[CLIENT] ---- Bandwidth summary ----")
        print(f"  elapsed: {elapsed:.2f} s")
        print(f"  sent: {bytes_sent} bytes ({bytes_sent * 8 / elapsed / 1e6:.3f} Mbps)")
        print(f"  received: {bytes_received} bytes ({bytes_received * 8 / elapsed / 1e6:.3f} Mbps)")
        print(f"  total: {total} bytes ({total * 8 / elapsed / 1e6:.3f} Mbps)")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
