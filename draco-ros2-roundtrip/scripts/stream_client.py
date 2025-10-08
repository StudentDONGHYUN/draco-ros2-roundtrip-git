#!/usr/bin/env python3
"""Stream PLY frames from rosbag: bag_to_ply -> draco encode -> send to server."""

from __future__ import annotations

import argparse
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
from typing import Dict, Optional


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
    payload = b''
    remaining = size
    while remaining:
        chunk = sock.recv(min(65536, remaining))
        if not chunk:
            raise ConnectionError("connection closed prematurely")
        payload += chunk
        remaining -= len(chunk)
    return name, payload


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


@dataclass
class FrameJob:
    seq: int
    name: str
    ply_bytes: bytes


def encode_worker(_worker_id: int,
                  encode_queue: "queue.Queue[Optional[FrameJob]]",
                  send_queue: "queue.Queue[Optional[tuple[int, str, Optional[bytes]]]]",
                  encoder: Path,
                  work_dir: Path,
                  cl: int,
                  qp: int,
                  qg: int,
                  extra: list[str],
                  _stop_event: threading.Event) -> None:
    while True:
        item = encode_queue.get()
        if item is None:
            encode_queue.task_done()
            break
        seq = item.seq
        name = item.name
        ply_bytes = item.ply_bytes
        try:
            drc_bytes = encode_ply_bytes(encoder, name, ply_bytes, work_dir, cl, qp, qg, extra)
            send_queue.put((seq, name, drc_bytes))
        except Exception as exc:
            print(f"[CLIENT] ENCODE FAIL {name}: {exc}")
            send_queue.put((seq, name, None))
        finally:
            encode_queue.task_done()


def sender_loop(sock: socket.socket,
                send_queue: "queue.Queue[Optional[tuple[int, str, Optional[bytes]]]]",
                stats: StreamingStats,
                stop_event: threading.Event,
                error_queue: "queue.Queue[Exception]") -> None:
    pending: Dict[int, tuple[str, Optional[bytes]]] = {}
    next_seq = 0
    try:
        while True:
            if stop_event.is_set() and send_queue.empty() and not pending:
                break
            item = send_queue.get()
            try:
                if item is None:
                    break
                seq, name, payload = item
                pending[seq] = (name, payload)
                while next_seq in pending:
                    pending_name, pending_payload = pending.pop(next_seq)
                    if pending_payload is None:
                        next_seq += 1
                        continue
                    message_name = pending_name.replace('.ply', '.drc')
                    try:
                        send_message(sock, message_name, pending_payload)
                    except Exception as exc:
                        error_queue.put(exc)
                        stop_event.set()
                        return
                    stats.add_sent(len(pending_payload))
                    print(f"[CLIENT] Sent {pending_name} ({len(pending_payload)} bytes)")
                    next_seq += 1
            finally:
                send_queue.task_done()
    finally:
        # Flush any remaining payloads in order when possible.
        while next_seq in pending and not stop_event.is_set():
            name, payload = pending.pop(next_seq)
            if payload is None:
                next_seq += 1
                continue
            message_name = name.replace('.ply', '.drc')
            try:
                send_message(sock, message_name, payload)
            except Exception as exc:
                error_queue.put(exc)
                stop_event.set()
                return
            stats.add_sent(len(payload))
            print(f"[CLIENT] Sent {name} ({len(payload)} bytes)")
            next_seq += 1


def receiver_loop(sock: socket.socket,
                  decoded_dir: Path,
                  stats: StreamingStats,
                  stop_event: threading.Event,
                  error_queue: "queue.Queue[Exception]",
                  decoded_files: set[Path]) -> None:
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
        name, payload = reply
        if name.startswith('decode-error:'):
            print(f"[CLIENT] Server decode error: {name}")
            continue
        out_path = decoded_dir / name
        try:
            out_path.write_bytes(payload)
        except Exception as exc:
            print(f"[CLIENT] WARN: failed to write {out_path.name}: {exc}")
        else:
            decoded_files.add(out_path)
        stats.add_received(len(payload))
        print(f"[CLIENT] Received {name} ({len(payload)} bytes)")


def ply_stream_reader(pipe: socket.socket,
                      encode_queue: "queue.Queue[Optional[FrameJob]]",
                      stop_event: threading.Event,
                      error_queue: "queue.Queue[Exception]") -> None:
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
                name = name_b.decode('utf-8')
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
                encode_queue.put(FrameJob(seq=seq, name=name, ply_bytes=bytes(payload)))
                seq += 1
    except Exception as exc:
        if not stop_event.is_set():
            error_queue.put(exc)
            stop_event.set()
    finally:
        try:
            pipe.close()
        except Exception:
            pass


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


def main():
    ap = argparse.ArgumentParser(description="Draco streaming client")
    ap.add_argument('--bag', required=True, help="rosbag directory")
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
    ap.add_argument('--encoder-workers', type=int, default=0,
                    help="Parallel Draco encoder workers (0 = auto)")
    ap.add_argument('--max-pending', type=int, default=16,
                    help="Max frames waiting to be encoded")
    args = ap.parse_args()

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
        # 자동 모드에서는 시스템 코어 수를 기반으로 하되 최소 1, 남는 코어를 1개 남겨둡니다.
        worker_count = max(1, cpu_count - 1)

    stats = StreamingStats()
    stop_event = threading.Event()
    error_queue: "queue.Queue[Exception]" = queue.Queue()
    encode_queue: "queue.Queue[Optional[FrameJob]]" = queue.Queue(maxsize=max_pending)
    send_queue: "queue.Queue[Optional[tuple[int, str, Optional[bytes]]]]" = queue.Queue()
    encode_threads: list[threading.Thread] = []
    stream_thread: Optional[threading.Thread] = None
    decoded_files: set[Path] = set()

    config_path = Path(__file__).resolve().parents[1] / 'configs' / 'qos_override.yaml'
    bag_cmd = ['ros2', 'bag', 'play', str(Path(args.bag).resolve())]
    if config_path.exists():
        bag_cmd += ['--qos-profile-overrides-path', str(config_path)]
    else:
        print(f"[CLIENT] WARN: QoS override file not found at {config_path}", file=sys.stderr)
    bag_process = subprocess.Popen(bag_cmd)

    stream_socket: Optional[socket.socket] = None
    stream_parent, stream_child = socket.socketpair()
    stream_socket = stream_parent
    try:
        saver_proc = launch_bag_to_ply(root, args, stream_fd=stream_child.fileno())
    except Exception:
        stream_parent.close()
        stream_socket = None
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
        args=(stream_socket, encode_queue, stop_event, error_queue),
        name="ply-stream",
        daemon=True,
    )
    stream_thread.start()

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
                args=(sock, send_queue, stats, stop_event, error_queue),
                name="sender",
                daemon=True,
            )
            receiver_thread = threading.Thread(
                target=receiver_loop,
                args=(sock, decoded_dir, stats, stop_event, error_queue, decoded_files),
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
                        stream_done = stream_thread is not None and not stream_thread.is_alive()
                        if stream_done and encode_queue.empty() and send_queue.empty():
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
                    send_message(sock, '', b'')
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
        if stream_thread and stream_thread.is_alive():
            stop_event.set()
            if stream_socket is not None:
                with suppress(Exception):
                    stream_socket.shutdown(socket.SHUT_RDWR)
                with suppress(Exception):
                    stream_socket.close()
            stream_thread.join(timeout=1.0)
        if stream_thread:
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

    elapsed = max(time.monotonic() - start_time, 1e-6)
    sent_bytes, received_bytes = stats.snapshot()
    total = sent_bytes + received_bytes
    print("[CLIENT] Streaming completed")
    print("[CLIENT] ---- Bandwidth summary ----")
    print(f"  elapsed: {elapsed:.2f} s")
    print(f"  sent: {sent_bytes} bytes ({sent_bytes * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  received: {received_bytes} bytes ({received_bytes * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  total: {total} bytes ({total * 8 / elapsed / 1e6:.3f} Mbps)")

    for path in decoded_files:
        with suppress(FileNotFoundError):
            path.unlink()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
