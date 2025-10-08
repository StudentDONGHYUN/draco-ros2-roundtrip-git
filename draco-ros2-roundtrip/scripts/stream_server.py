#!/usr/bin/env python3
"""Standalone Draco streaming server with parallel decoder workers."""

from __future__ import annotations

import argparse
import os
import socket
import struct
import subprocess
import sys
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Lock, Thread
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


_HEADER = struct.Struct('!I')
_SIZE = struct.Struct('!Q')
_SEP = ':'

MSG_DATA = 'data'
MSG_ERROR = 'error'
MSG_EOF = 'eof'


@dataclass
class Message:
    kind: str
    name: str
    payload: bytes

    def as_meta(self) -> str:
        return f"{self.kind}{_SEP}{self.name}" if self.name else self.kind

    @classmethod
    def from_meta(cls, meta: str, payload: bytes) -> "Message":
        if _SEP in meta:
            kind, name = meta.split(_SEP, 1)
        else:
            kind, name = MSG_DATA, meta
        return cls(kind=kind or MSG_DATA, name=name, payload=payload)


def _read_exact(sock: socket.socket, size: int) -> bytes:
    buf = bytearray()
    while len(buf) < size:
        chunk = sock.recv(size - len(buf))
        if not chunk:
            raise ConnectionError("socket closed while reading")
        buf.extend(chunk)
    return bytes(buf)


def recv_message(sock: socket.socket) -> Optional[Message]:
    header = sock.recv(_HEADER.size)
    if not header:
        return None
    if len(header) != _HEADER.size:
        raise RuntimeError("incomplete header")
    (name_len,) = _HEADER.unpack(header)
    if name_len <= 0:
        return None
    meta = _read_exact(sock, name_len).decode('utf-8')
    (payload_len,) = _SIZE.unpack(_read_exact(sock, _SIZE.size))
    payload = _read_exact(sock, payload_len) if payload_len else b''
    return Message.from_meta(meta, payload)


def send_message(sock: socket.socket, message: Message) -> None:
    meta = message.as_meta().encode('utf-8')
    sock.sendall(_HEADER.pack(len(meta)))
    sock.sendall(meta)
    sock.sendall(_SIZE.pack(len(message.payload)))
    if message.payload:
        sock.sendall(message.payload)


@dataclass
class DecodeJob:
    seq: int
    stem: str
    payload: bytes


@dataclass
class DecodeResult:
    seq: int
    stem: str
    data: Optional[bytes]
    error: Optional[str]


def decode_drc(decoder: Path, drc_bytes: bytes, out_dir: Path, stem: str, seq: int) -> bytes:
    unique = f"{stem}_{seq}"
    drc_path = out_dir / f"{unique}.drc"
    ply_path = out_dir / f"{unique}.decoded.ply"
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


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Draco streaming server")
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--decoder', default=None)
    ap.add_argument('--work-dir', default='data/server_tmp')
    ap.add_argument('--decoder-workers', type=int, default=0,
                    help="Parallel decoder workers (0 = auto)")
    return ap.parse_args()


def main() -> None:
    args = parse_args()

    decoder = find_exe('draco_decoder', args.decoder)
    work_dir = Path(args.work_dir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)

    cpu_count = os.cpu_count() or 1
    worker_count = args.decoder_workers if args.decoder_workers > 0 else max(1, min(4, cpu_count))

    decode_queue: "Queue[Optional[DecodeJob]]" = Queue(maxsize=worker_count * 2)
    result_queue: "Queue[Optional[DecodeResult]]" = Queue()
    stop_event = Event()
    bytes_lock = Lock()
    bytes_in = 0
    bytes_out = 0

    def worker_loop() -> None:
        while True:
            job = decode_queue.get()
            if job is None:
                decode_queue.task_done()
                break
            try:
                ply_bytes = decode_drc(decoder, job.payload, work_dir, job.stem, job.seq)
                result_queue.put(DecodeResult(seq=job.seq, stem=job.stem, data=ply_bytes, error=None))
            except Exception as exc:  # pylint: disable=broad-except
                result_queue.put(DecodeResult(seq=job.seq, stem=job.stem, data=None, error=str(exc)))
            finally:
                decode_queue.task_done()

    workers = [Thread(target=worker_loop, name=f"decoder-{i}", daemon=True) for i in range(worker_count)]
    for t in workers:
        t.start()

    start_time = time.monotonic()

    with socket.create_server((args.host, args.port), reuse_port=True) as server:
        print(f"[SERVER] Listening on {args.host}:{args.port}")
        conn, addr = server.accept()
        print(f"[SERVER] Connection from {addr}")

        total_jobs = 0

        def sender_loop() -> None:
            nonlocal bytes_out
            nonlocal total_jobs
            pending: Dict[int, DecodeResult] = {}
            next_seq = 0
            processed = 0
            while not stop_event.is_set():
                try:
                    item = result_queue.get(timeout=0.1)
                except Empty:
                    if processed >= total_jobs and decode_queue.empty():
                        break
                    continue
                if item is None:
                    result_queue.task_done()
                    break
                pending[item.seq] = item
                while next_seq in pending:
                    result = pending.pop(next_seq)
                    if result.error is not None:
                        msg = Message(kind=MSG_ERROR, name=result.stem, payload=result.error.encode())
                    else:
                        payload = result.data or b''
                        msg = Message(kind=MSG_DATA, name=f"{result.stem}.decoded.ply", payload=payload)
                        with bytes_lock:
                            bytes_out += len(payload)
                    try:
                        send_message(conn, msg)
                    except Exception as exc:  # pylint: disable=broad-except
                        print(f"[SERVER] ERROR sending {result.stem}: {exc}")
                        stop_event.set()
                        break
                    next_seq += 1
                    processed += 1
                result_queue.task_done()
                if processed >= total_jobs and decode_queue.empty() and not pending:
                    break

        with conn:
            sender_thread = Thread(target=sender_loop, daemon=True)
            sender_thread.start()
            seq = 0
            try:
                while not stop_event.is_set():
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
                    base_stem, _, _meta = stem_raw.partition('|')
                    stem = Path(base_stem).stem
                    with bytes_lock:
                        bytes_in += len(msg.payload)
                    print(f"[SERVER] Received {stem_raw} ({len(msg.payload)} bytes)")
                    decode_queue.put(DecodeJob(seq=seq, stem=stem, payload=msg.payload))
                    seq += 1
                    total_jobs += 1
            finally:
                stop_event.set()
                for _ in workers:
                    decode_queue.put(None)
                decode_queue.join()
                result_queue.put(None)
                sender_thread.join()
                for t in workers:
                    t.join()
                with suppress(Exception):
                    send_message(conn, Message(kind=MSG_EOF, name='', payload=b''))

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
