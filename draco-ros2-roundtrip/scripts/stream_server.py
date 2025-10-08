#!/usr/bin/env python3
"""Receive Draco .drc files over TCP, decode to PLY, send back to client."""

import argparse
import socket
import struct
import subprocess
import sys
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


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


def recv_n(sock: socket.socket, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("connection closed prematurely")
        data.extend(chunk)
    return bytes(data)


MSG_DATA = 'data'
MSG_ERROR = 'error'
MSG_EOF = 'eof'
_SEPARATOR = ':'


@dataclass
class Message:
    kind: str
    name: str
    payload: bytes

    def as_meta(self) -> str:
        return f"{self.kind}{_SEPARATOR}{self.name}" if self.name else self.kind

    @classmethod
    def from_meta(cls, meta: str, payload: bytes) -> "Message":
        if _SEPARATOR in meta:
            kind, name = meta.split(_SEPARATOR, 1)
        else:
            kind, name = MSG_DATA, meta
        return cls(kind=kind or MSG_DATA, name=name, payload=payload)


def recv_message(sock: socket.socket) -> Optional[Message]:
    header = sock.recv(4)
    if not header:
        return None
    if len(header) != 4:
        raise RuntimeError('incomplete header')
    name_len = struct.unpack('!I', header)[0]
    if name_len <= 0:
        return None
    meta = recv_n(sock, name_len).decode('utf-8')
    size = struct.unpack('!Q', recv_n(sock, 8))[0]
    payload = recv_n(sock, size) if size else b''
    return Message.from_meta(meta, payload)


def send_message(sock: socket.socket, message: Message) -> None:
    meta = message.as_meta().encode('utf-8')
    sock.sendall(struct.pack('!I', len(meta)))
    sock.sendall(meta)
    sock.sendall(struct.pack('!Q', len(message.payload)))
    if message.payload:
        sock.sendall(message.payload)


def decode_drc(decoder: Path, drc_bytes: bytes, out_dir: Path, stem: str) -> bytes:
    out_dir.mkdir(parents=True, exist_ok=True)
    drc_path = out_dir / f"{stem}.drc"
    ply_path = out_dir / f"{stem}.decoded.ply"
    drc_path.write_bytes(drc_bytes)
    cmd = [str(decoder), "-i", str(drc_path), "-o", str(ply_path)]
    rc = subprocess.run(cmd, capture_output=True, text=True)
    if rc.returncode != 0:
        raise RuntimeError(f"draco_decoder failed: {rc.stderr.strip()}")
    data = ply_path.read_bytes()
    with suppress(FileNotFoundError):
        drc_path.unlink()
    with suppress(FileNotFoundError):
        ply_path.unlink()
    return data


def main():
    ap = argparse.ArgumentParser(description="Draco streaming server")
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--decoder', default=None, help="Path to draco_decoder")
    ap.add_argument('--work-dir', default='data/server_tmp')
    args = ap.parse_args()

    decoder = find_exe('draco_decoder', args.decoder)
    work_dir = Path(args.work_dir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)

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
                if msg.kind == MSG_EOF:
                    print("[SERVER] EOF received")
                    break
                if msg.kind != MSG_DATA:
                    print(f"[SERVER] Ignoring unexpected message kind: {msg.kind}")
                    continue
                stem_raw = msg.name or "frame"
                base_stem, _, _frame_meta = stem_raw.partition('|')
                stem = Path(base_stem).stem
                payload = msg.payload
                bytes_in += len(payload)
                print(f"[SERVER] Received {stem_raw} ({len(payload)} bytes)")
                try:
                    ply_bytes = decode_drc(decoder, payload, work_dir, stem)
                except Exception as exc:
                    error_msg = Message(kind=MSG_ERROR, name=stem_raw, payload=str(exc).encode())
                    send_message(conn, error_msg)
                    print(f"[SERVER] ERROR {exc}")
                    continue
                send_name = f"{stem}.decoded.ply"
                send_message(conn, Message(kind=MSG_DATA, name=send_name, payload=ply_bytes))
                bytes_out += len(ply_bytes)
                print(f"[SERVER] Sent {send_name} ({len(ply_bytes)} bytes)")

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
