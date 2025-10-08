#!/usr/bin/env python3
"""Stream rosbag frames through Draco encode/transfer with live playback."""

from __future__ import annotations

import argparse
import io
import os
import queue
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from draco_roundtrip.utils import (
    Message,
    MSG_DATA,
    MSG_EOF,
    MSG_ERROR,
    recv_message,
    send_message,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - during setup or if ament not available
    get_package_share_directory = None  # type: ignore

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
    data = drc_path.read_bytes()
    with suppress(FileNotFoundError):
        drc_path.unlink()
    return data


def encode_ply_bytes(encoder: Path,
                     ply_name: str,
                     ply_bytes: bytes,
                     work_dir: Path,
                     cl: int,
                     qp: int,
                     qg: int,
                     extra: list[str]) -> bytes:
    fd, tmp_path = tempfile.mkstemp(prefix=Path(ply_name).stem + '_', suffix='.ply', dir=str(work_dir))
    try:
        with os.fdopen(fd, 'wb') as tmp_file:
            tmp_file.write(ply_bytes)
        try:
            return encode_ply(encoder, Path(tmp_path), work_dir, cl, qp, qg, extra)
        finally:
            with suppress(FileNotFoundError):
                Path(tmp_path).unlink()
    except Exception:
        with suppress(FileNotFoundError):
            Path(tmp_path).unlink()
        raise


class StreamingStats:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._bytes_sent = 0
        self._bytes_received = 0

    def add_sent(self, amount: int) -> None:
        with self._lock:
            self._bytes_sent += amount

    def add_received(self, amount: int) -> None:
        with self._lock:
            self._bytes_received += amount

    def snapshot(self) -> tuple[int, int]:
        with self._lock:
            return self._bytes_sent, self._bytes_received


def load_points_from_bytes(data: bytes) -> np.ndarray:
    ply = PlyData.read(io.BytesIO(data))
    verts = ply['vertex']
    arr = np.vstack((verts['x'], verts['y'], verts['z']))
    return arr.T.astype(np.float32)


def compute_metrics(src_pts: np.ndarray, dec_pts: np.ndarray, sample: int) -> tuple[int, int, int, np.ndarray, float, np.ndarray, str, str]:
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


@dataclass
class FrameJob:
    seq: int
    name: str
    ply_bytes: bytes
    src_points: np.ndarray
    frame_id: str


@dataclass
class FrameContext:
    seq: int
    name: str
    ply_bytes: bytes
    src_points: np.ndarray
    frame_id: str


class PlaybackNode(Node):
    def __init__(self, queue_obj: queue.Queue, default_frame_id: str,
                 topic_prefix: str, hz: float, tf_parent: str,
                 broadcast_tf: bool):
        super().__init__('stream_playback')
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_src = self.create_publisher(PointCloud2, f"/{topic_prefix}/source", qos)
        self.pub_dec = self.create_publisher(PointCloud2, f"/{topic_prefix}/decoded", qos)
        self.queue = queue_obj
        self.default_frame_id = default_frame_id
        self.tf_parent = tf_parent
        self.broadcast_tf = broadcast_tf
        self.tf_broadcaster = StaticTransformBroadcaster(self) if broadcast_tf else None
        self.broadcasted_frames: set[str] = set()
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
        frame_idx, name, frame_id, pts_src, pts_dec = item
        frame_id = frame_id or self.default_frame_id
        if not frame_id:
            frame_id = self.tf_parent
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg_src = pc2.create_cloud_xyz32(header, pts_src)
        msg_dec = pc2.create_cloud_xyz32(header, pts_dec)
        self.pub_src.publish(msg_src)
        self.pub_dec.publish(msg_dec)
        if (self.broadcast_tf and self.tf_broadcaster and frame_id not in self.broadcasted_frames
                and frame_id != self.tf_parent):
            transform = TransformStamped()
            transform.header.stamp = header.stamp
            transform.header.frame_id = self.tf_parent
            transform.child_frame_id = frame_id
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.w = 1.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            self.tf_broadcaster.sendTransform(transform)
            self.broadcasted_frames.add(frame_id)
        self.get_logger().debug(f"Published frame {frame_idx}: {name}")


def start_playback_thread(queue_obj: queue.Queue, default_frame_id: str,
                          topic_prefix: str, hz: float,
                          tf_parent: str, broadcast_tf: bool) -> threading.Thread:
    def _run():
        rclpy.init()
        node = PlaybackNode(queue_obj, default_frame_id, topic_prefix, hz,
                             tf_parent=tf_parent, broadcast_tf=broadcast_tf)
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


def encode_worker(_worker_id: int,
                  encode_queue: "queue.Queue[Optional[FrameJob]]",
                  send_queue: "queue.Queue[Optional[tuple[int, str, Optional[bytes], FrameJob]]]",
                  encoder: Path,
                  work_dir: Path,
                  cl: int,
                  qp: int,
                  qg: int,
                  extra: list[str],
                  _stop_event: threading.Event) -> None:
    while True:
        job = encode_queue.get()
        if job is None:
            encode_queue.task_done()
            break
        try:
            drc_bytes = encode_ply_bytes(encoder, job.name, job.ply_bytes, work_dir, cl, qp, qg, extra)
            send_queue.put((job.seq, job.name, drc_bytes, job))
        except Exception as exc:
            print(f"[CLIENT] ENCODE FAIL {job.name}: {exc}")
            send_queue.put((job.seq, job.name, None, job))
        finally:
            encode_queue.task_done()


def sender_loop(sock: socket.socket,
                send_queue: "queue.Queue[Optional[tuple[int, str, Optional[bytes], FrameJob]]]",
                stats: StreamingStats,
                stop_event: threading.Event,
                error_queue: "queue.Queue[Exception]",
                context_store: Dict[str, FrameContext],
                context_lock: threading.Lock) -> None:
    pending: Dict[int, tuple[str, Optional[bytes], FrameJob]] = {}
    next_seq = 0
    try:
        while True:
            if stop_event.is_set() and send_queue.empty() and not pending:
                break
            item = send_queue.get()
            try:
                if item is None:
                    break
                seq, name, payload, job = item
                pending[seq] = (name, payload, job)
                while next_seq in pending:
                    pending_name, pending_payload, pending_job = pending.pop(next_seq)
                    if pending_payload is None:
                        next_seq += 1
                        continue
                    message_name = pending_name.replace('.ply', '.drc')
                    try:
                        send_message(sock, Message(kind=MSG_DATA, name=message_name, payload=pending_payload))
                    except Exception as exc:
                        error_queue.put(exc)
                        stop_event.set()
                        return
                    stats.add_sent(len(pending_payload))
                    stem = pending_name.split('.', 1)[0]
                    with context_lock:
                        context_store[stem] = FrameContext(
                            seq=pending_job.seq,
                            name=pending_job.name,
                            ply_bytes=pending_job.ply_bytes,
                            src_points=pending_job.src_points,
                            frame_id=pending_job.frame_id,
                        )
                    print(f"[CLIENT] Sent {pending_name} ({len(pending_payload)} bytes)")
                    next_seq += 1
            finally:
                send_queue.task_done()
    finally:
        while next_seq in pending and not stop_event.is_set():
            name, payload, job = pending.pop(next_seq)
            if payload is None:
                next_seq += 1
                continue
            message_name = name.replace('.ply', '.drc')
            try:
                send_message(sock, Message(kind=MSG_DATA, name=message_name, payload=payload))
            except Exception as exc:
                error_queue.put(exc)
                stop_event.set()
                return
            stats.add_sent(len(payload))
            stem = name.split('.', 1)[0]
            with context_lock:
                context_store[stem] = FrameContext(seq=job.seq, name=job.name,
                                                   ply_bytes=job.ply_bytes, src_points=job.src_points,
                                                   frame_id=job.frame_id)
            print(f"[CLIENT] Sent {name} ({len(payload)} bytes)")
            next_seq += 1


def receiver_loop(sock: socket.socket,
                  decoded_dir: Path,
                  stats: StreamingStats,
                  stop_event: threading.Event,
                  error_queue: "queue.Queue[Exception]",
                  context_store: Dict[str, FrameContext],
                  context_lock: threading.Lock,
                  play_queue: queue.Queue,
                  play_sample: int,
                  decoded_files: set[Path],
                  print_metrics: bool) -> None:
    frame_idx = 0
    next_play_seq = 0
    pending_decoded: Dict[int, tuple[str, FrameContext, np.ndarray, str, int]] = {}
    while not stop_event.is_set():
        try:
            reply = recv_message(sock)
        except Exception as exc:
            if not stop_event.is_set():
                error_queue.put(exc)
                stop_event.set()
            return
        if reply is None:
            break
        if reply.kind == MSG_EOF:
            break
        if reply.kind == MSG_ERROR:
            print(f"[CLIENT] Server decode error: {reply.name}: {reply.payload.decode('utf-8', 'ignore')}")
            continue
        if reply.kind != MSG_DATA:
            print(f"[CLIENT] WARN: unexpected message kind {reply.kind}")
            continue
        name = reply.name or "frame"
        payload = reply.payload
        stats.add_received(len(payload))
        out_path = decoded_dir / name
        try:
            out_path.write_bytes(payload)
        except Exception as exc:
            print(f"[CLIENT] WARN: failed to write {out_path.name}: {exc}")
        else:
            decoded_files.add(out_path)

        stem_base = Path(name).stem
        stem = stem_base.split('.decoded', 1)[0]
        with context_lock:
            ctx = context_store.get(stem)
        if ctx is None:
            print(f"[CLIENT] WARN: missing context for {name}")
            continue

        try:
            dec_pts = load_points_from_bytes(payload)
        except Exception as exc:
            print(f"[CLIENT] WARN: failed to parse decoded payload for {name}: {exc}")
            continue

        pending_decoded[ctx.seq] = (stem, ctx, dec_pts, name, len(payload))

        while next_play_seq in pending_decoded:
            stem_key, ctx_ready, dec_pts_ready, resp_name, payload_len = pending_decoded.pop(next_play_seq)
            frame_idx += 1
            metrics = compute_metrics(ctx_ready.src_points, dec_pts_ready, play_sample)
            if print_metrics:
                n_src, n_dec, diff, centroid_delta, centroid_norm, bbox_delta, mean_str, max_str = metrics
                print(f"[FRAME {frame_idx:04d}] {ctx_ready.name.split('.', 1)[0]}")
                print(
                    "  points:  "
                    f"src={n_src}  dec={n_dec}  diff={diff:+d}"
                )
                print(
                    "  centroid diff (m):  "
                    f"x={centroid_delta[0]:+.3f}  y={centroid_delta[1]:+.3f}  z={centroid_delta[2]:+.3f}  ‖Δ‖={centroid_norm:.3f}"
                )
                print(
                    "  bbox size Δ (m):   "
                    f"x={bbox_delta[0]:+.3f}  y={bbox_delta[1]:+.3f}  z={bbox_delta[2]:+.3f}"
                )
                print(f"  mean|max dist (m): {mean_str}/{max_str}")
            play_queue.put(
                (frame_idx,
                 ctx_ready.name.split('.', 1)[0],
                 ctx_ready.frame_id,
                 ctx_ready.src_points.astype(np.float32),
                 dec_pts_ready.astype(np.float32)))
            print(f"[CLIENT] Received {resp_name} ({payload_len} bytes)")
            with context_lock:
                context_store.pop(stem_key, None)
            next_play_seq += 1


def ply_stream_reader(pipe: socket.socket,
                      encode_queue: "queue.Queue[Optional[FrameJob]]",
                      stop_event: threading.Event,
                      error_queue: "queue.Queue[Exception]",
                      default_frame_id: str) -> None:
    seq = 0
    try:
        with pipe, pipe.makefile('rb') as fh:
            while not stop_event.is_set():
                header = fh.read(4)
                if not header:
                    break
                if len(header) < 4:
                    raise ConnectionError("Incomplete frame header from stream")
                name_len = struct.unpack('!I', header)[0]
                if name_len == 0:
                    break
                name_b = fh.read(name_len)
                if len(name_b) < name_len:
                    raise ConnectionError("Stream closed while reading name")
                raw_name = name_b.decode('utf-8')
                base_name, sep, frame_meta = raw_name.partition('|')
                frame_id = frame_meta if sep else default_frame_id
                if not frame_id:
                    frame_id = default_frame_id
                name = base_name
                size_bytes = fh.read(8)
                if len(size_bytes) < 8:
                    raise ConnectionError("Stream closed while reading payload size")
                size = struct.unpack('!Q', size_bytes)[0]
                payload = bytearray()
                remaining = size
                while remaining:
                    chunk = fh.read(min(65536, remaining))
                    if not chunk:
                        raise ConnectionError("Stream closed while reading payload")
                    payload.extend(chunk)
                    remaining -= len(chunk)
                try:
                    src_pts = load_points_from_bytes(bytes(payload))
                except Exception as exc:
                    error_queue.put(exc)
                    stop_event.set()
                    return
                encode_queue.put(FrameJob(seq=seq, name=name, ply_bytes=bytes(payload),
                                           src_points=src_pts, frame_id=frame_id))
                seq += 1
    except Exception as exc:
        if not stop_event.is_set():
            error_queue.put(exc)
            stop_event.set()
    finally:
        with suppress(Exception):
            pipe.close()


def launch_bag_to_ply(root: Path,
                      args: argparse.Namespace,
                      stream_fd: Optional[int] = None) -> subprocess.Popen:
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
    pass_fds = ()
    if stream_fd is not None:
        cmd += ['--stream-fd', str(stream_fd)]
        pass_fds = (stream_fd,)
    return subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr, pass_fds=pass_fds)


def main(argv: Iterable[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Streaming client with live playback (async)")
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
    ap.add_argument('--tf-parent-frame', default='map',
                    help="Parent frame used when broadcasting static TF for streamed clouds")
    ap.add_argument('--no-tf', action='store_true',
                    help="Disable automatic TF broadcasting for streamed frames")
    ap.add_argument('--print-metrics', action='store_true',
                    help="Print per-frame metrics when decoding frames")
    ap.add_argument('--encoder-workers', type=int, default=0,
                    help="Parallel Draco encoder workers (0 = auto)")
    ap.add_argument('--max-pending', type=int, default=16,
                    help="Max frames waiting to be encoded")
    args = ap.parse_args(argv)

    encoder = find_exe('draco_encoder', args.encoder)
    root = Path(__file__).resolve().parents[1]

    ply_dir = Path(args.ply_dir).resolve()
    ply_dir.mkdir(parents=True, exist_ok=True)
    work_dir = Path(args.work_dir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    decoded_dir = Path(args.decoded_dir).resolve()
    decoded_dir.mkdir(parents=True, exist_ok=True)

    max_pending = max(1, args.max_pending)
    cpu_count = os.cpu_count() or 1
    if args.encoder_workers > 0:
        worker_count = args.encoder_workers
    else:
        worker_count = max(1, cpu_count - 1)

    stats = StreamingStats()
    stop_event = threading.Event()
    error_queue: "queue.Queue[Exception]" = queue.Queue()
    encode_queue: "queue.Queue[Optional[FrameJob]]" = queue.Queue(maxsize=max_pending)
    send_queue: "queue.Queue[Optional[tuple[int, str, Optional[bytes], FrameJob]]]" = queue.Queue()
    context_store: Dict[str, FrameContext] = {}
    context_lock = threading.Lock()

    config_path = resolve_qos_override()
    bag_cmd = ['ros2', 'bag', 'play', str(Path(args.bag).resolve())]
    if config_path is not None:
        bag_cmd += ['--qos-profile-overrides-path', str(config_path)]
    else:
        print('[CLIENT] WARN: QoS override file not found, falling back to recorded QoS', file=sys.stderr)
    bag_process = subprocess.Popen(bag_cmd)

    stream_parent, stream_child = socket.socketpair()
    try:
        saver_proc = launch_bag_to_ply(root, args, stream_fd=stream_child.fileno())
    except Exception:
        stream_parent.close()
        stream_child.close()
        raise
    finally:
        with suppress(Exception):
            stream_child.close()

    encode_threads = [
        threading.Thread(
            target=encode_worker,
            args=(idx, encode_queue, send_queue, encoder, work_dir,
                  args.cl, args.qp, args.qg, args.encoder_extra, stop_event),
            name=f"encoder-{idx}",
            daemon=True,
        )
        for idx in range(worker_count)
    ]
    for thread in encode_threads:
        thread.start()

    stream_thread = threading.Thread(
        target=ply_stream_reader,
        args=(stream_parent, encode_queue, stop_event, error_queue, args.play_frame_id),
        name="ply-stream",
        daemon=True,
    )
    stream_thread.start()

    to_play: queue.Queue = queue.Queue()
    playback_thread = start_playback_thread(
        to_play,
        args.play_frame_id,
        args.play_topic_prefix,
        args.play_hz,
        args.tf_parent_frame,
        not args.no_tf,
    )
    decoded_files: set[Path] = set()

    start_time = time.monotonic()
    sender_thread: Optional[threading.Thread] = None
    receiver_thread: Optional[threading.Thread] = None
    encode_shutdown = False
    sender_shutdown = False

    try:
        with socket.create_connection((args.server_host, args.server_port)) as sock:
            print(f"[CLIENT] Connected to {args.server_host}:{args.server_port}")
            sender_thread = threading.Thread(
                target=sender_loop,
                args=(sock, send_queue, stats, stop_event, error_queue, context_store, context_lock),
                name="sender",
                daemon=True,
            )
            receiver_thread = threading.Thread(
                target=receiver_loop,
                args=(sock, decoded_dir, stats, stop_event, error_queue, context_store, context_lock,
                      to_play, args.play_sample, decoded_files, args.print_metrics),
                name="receiver",
                daemon=True,
            )
            receiver_thread.start()
            sender_thread.start()

            try:
                while not stop_event.is_set():
                    try:
                        exc = error_queue.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        raise exc

                    if saver_proc.poll() is not None and bag_process.poll() is not None:
                        stream_done = not stream_thread.is_alive()
                        with context_lock:
                            pending_contexts = bool(context_store)
                        if stream_done and encode_queue.empty() and send_queue.empty() and not pending_contexts:
                            break
                    time.sleep(0.05)
            finally:
                if not stop_event.is_set():
                    encode_queue.join()
                else:
                    while True:
                        try:
                            encode_queue.get_nowait()
                        except queue.Empty:
                            break
                        else:
                            encode_queue.task_done()
                for _ in encode_threads:
                    encode_queue.put(None)
                encode_shutdown = True
                for thread in encode_threads:
                    thread.join()

                if sender_thread:
                    send_queue.put(None)
                    sender_shutdown = True
                    send_queue.join()
                    sender_thread.join()

                with suppress(Exception):
                    send_message(sock, Message(kind=MSG_EOF, name='', payload=b''))
                stop_event.set()
                with suppress(OSError):
                    sock.shutdown(socket.SHUT_RD)
                if receiver_thread:
                    receiver_thread.join()
    except Exception:
        stop_event.set()
        raise
    finally:
        if not encode_shutdown:
            for _ in encode_threads:
                encode_queue.put(None)
        for thread in encode_threads:
            thread.join(timeout=1.0)
        if sender_thread and not sender_shutdown:
            send_queue.put(None)
        if sender_thread:
            sender_thread.join(timeout=1.0)
        if receiver_thread and receiver_thread.is_alive():
            stop_event.set()
            receiver_thread.join(timeout=1.0)
        if stream_thread.is_alive():
            stop_event.set()
            with suppress(Exception):
                stream_parent.shutdown(socket.SHUT_RDWR)
            stream_thread.join(timeout=1.0)
        if stop_event.is_set():
            with suppress(Exception):
                if bag_process.poll() is None:
                    bag_process.terminate()
            with suppress(Exception):
                if saver_proc.poll() is None:
                    saver_proc.terminate()
        saver_proc.wait()
        bag_process.wait()

    to_play.put(None)
    playback_thread.join(timeout=5)

    pending_error: Optional[Exception] = None
    try:
        pending_error = error_queue.get_nowait()
    except queue.Empty:
        pending_error = None
    if pending_error:
        for path in decoded_files:
            with suppress(FileNotFoundError):
                path.unlink()
        raise pending_error

    for path in decoded_files:
        with suppress(FileNotFoundError):
            path.unlink()

    elapsed = max(time.monotonic() - start_time, 1e-6)
    sent_bytes, received_bytes = stats.snapshot()
    total = sent_bytes + received_bytes
    print("[CLIENT] Streaming playback completed")
    print("[CLIENT] ---- Bandwidth summary ----")
    print(f"  elapsed: {elapsed:.2f} s")
    print(f"  sent: {sent_bytes} bytes ({sent_bytes * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  received: {received_bytes} bytes ({received_bytes * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  total: {total} bytes ({total * 8 / elapsed / 1e6:.3f} Mbps)")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
