#!/usr/bin/env python3
"""Stream PLY frames from rosbag: bag_to_ply -> draco encode -> send to server."""

from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Set


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
    return drc_path.read_bytes()


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
    args = ap.parse_args()

    encoder = find_exe('draco_encoder', args.encoder)
    root = Path(__file__).resolve().parents[1]

    ply_dir = Path(args.ply_dir).resolve()
    ply_dir.mkdir(parents=True, exist_ok=True)
    work_dir = Path(args.work_dir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    decoded_dir = Path(args.decoded_dir).resolve()
    decoded_dir.mkdir(parents=True, exist_ok=True)

    config_path = Path(__file__).resolve().parents[1] / 'configs' / 'qos_override.yaml'
    bag_cmd = ['ros2', 'bag', 'play', str(Path(args.bag).resolve())]
    if config_path.exists():
        bag_cmd += ['--qos-profile-overrides-path', str(config_path)]
    else:
        print(f"[CLIENT] WARN: QoS override file not found at {config_path}", file=sys.stderr)
    bag_process = subprocess.Popen(bag_cmd)
    saver_proc = launch_bag_to_ply(root, args)

    processed: Set[Path] = set()

    start_time = time.monotonic()
    bytes_sent = 0
    bytes_received = 0

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
                        bytes_received += len(payload)
                        print(f"[CLIENT] Received {name} ({len(payload)} bytes)")
                    processed.add(ply_path)
                if saver_proc.poll() is not None and bag_process.poll() is not None:
                    if len(processed) == len(list(ply_dir.glob(f"{args.prefix}_*.ply"))):
                        break
                time.sleep(0.2)
        finally:
            send_message(sock, '', b'')
    saver_proc.wait()
    bag_process.wait()
    print("[CLIENT] Streaming completed")
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
