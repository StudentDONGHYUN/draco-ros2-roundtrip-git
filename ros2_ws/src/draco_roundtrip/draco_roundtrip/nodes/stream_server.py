#!/usr/bin/env python3
"""Receive Draco .drc files over TCP, decode to PLY, send back to client."""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
from contextlib import suppress
from pathlib import Path

from draco_roundtrip.utils import (
    Message,
    MSG_DATA,
    MSG_ERROR,
    ensure_directory,
    recv_message,
    resolve_executable,
    send_message,
)


def decode_drc(decoder: Path, drc_bytes: bytes, out_dir: Path, stem: str) -> bytes:
    ensure_directory(out_dir)
    drc_path = out_dir / f"{stem}.drc"
    ply_path = out_dir / f"{stem}.decoded.ply"
    drc_path.write_bytes(drc_bytes)
    cmd = [str(decoder), "-i", str(drc_path), "-o", str(ply_path)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"draco_decoder failed (rc={proc.returncode}):\n"
            f"STDOUT: {proc.stdout.strip()}\nSTDERR: {proc.stderr.strip()}"
        )
    data = ply_path.read_bytes()
    with suppress(FileNotFoundError):
        drc_path.unlink()
    with suppress(FileNotFoundError):
        ply_path.unlink()
    return data


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Draco streaming server")
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--decoder', default=None, help="Path to draco_decoder")
    ap.add_argument('--work-dir', default='data/server_tmp')
    return ap


def main(argv: list[str] | None = None) -> None:
    args = build_arg_parser().parse_args(argv)

    decoder = resolve_executable('draco_decoder', args.decoder, env_var='DRACO_DECODER')
    work_dir = ensure_directory(Path(args.work_dir).resolve())

    start_time = time.monotonic()
    bytes_in = 0
    bytes_out = 0

    with socket.create_server((args.host, args.port), reuse_port=True) as server:
        print(f"[SERVER] Listening on {args.host}:{args.port}")
        conn, addr = server.accept()
        print(f"[SERVER] Connection from {addr}")
        with conn:
            while True:
                msg = recv_message(conn)
                if msg is None:
                    print("[SERVER] End of stream")
                    break
                if msg.kind != MSG_DATA:
                    print(f"[SERVER] Ignoring unexpected message kind: {msg.kind}")
                    continue
                stem = msg.name or "frame"
                bytes_in += len(msg.payload)
                print(f"[SERVER] Received {stem} ({len(msg.payload)} bytes)")
                try:
                    ply_bytes = decode_drc(decoder, msg.payload, work_dir, stem)
                except Exception as exc:
                    error_msg = Message(kind=MSG_ERROR, name=stem, payload=str(exc).encode())
                    send_message(conn, error_msg)
                    print(f"[SERVER] ERROR decoding {stem}: {exc}")
                    continue
                reply = Message(kind=MSG_DATA, name=f"{stem}.decoded", payload=ply_bytes)
                send_message(conn, reply)
                bytes_out += len(ply_bytes)
                print(f"[SERVER] Sent {reply.name} ({len(ply_bytes)} bytes)")

    elapsed = max(time.monotonic() - start_time, 1e-6)
    total = bytes_in + bytes_out
    print("[SERVER] ---- Bandwidth summary ----")
    print(f"  elapsed: {elapsed:.2f} s")
    print(f"  inbound: {bytes_in} bytes ({bytes_in * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  outbound: {bytes_out} bytes ({bytes_out * 8 / elapsed / 1e6:.3f} Mbps)")
    print(f"  total: {total} bytes ({total * 8 / elapsed / 1e6:.3f} Mbps)")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
