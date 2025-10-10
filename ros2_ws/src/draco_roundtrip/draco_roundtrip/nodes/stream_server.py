#!/usr/bin/env python3
"""Receive Draco .drc files over TCP, decode with worker pool, send back to client."""

from __future__ import annotations

import argparse
import atexit
import os
import socket
import subprocess
import sys
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
from threading import BoundedSemaphore, Event, Lock, Thread
from typing import Any, Dict, Optional

from draco_roundtrip.utils import (
    Message,
    MSG_DATA,
    MSG_EOF,
    MSG_ERROR,
    ensure_directory,
    create_monitor,
    recommended_worker_count,
    recv_message,
    resolve_executable,
    send_message,
)

RECV_TIMEOUT_SEC = 0.5


_MONITOR = create_monitor("server")
_DEBUG_PRINT = os.environ.get("DRACO_STREAM_DEBUG", "").lower() in {"1", "true", "yes", "on"}


def monitor_event(event: str, **payload: Any) -> None:
    _MONITOR.emit(event, **payload)
    if _DEBUG_PRINT:
        extras = " ".join(f"{key}={payload[key]}" for key in sorted(payload))
        print(f"[DEBUG][server:{event}] {extras}", flush=True)


atexit.register(_MONITOR.close)


def shutdown_socket(sock: Optional[socket.socket]) -> None:
    if sock is None:
        return
    with suppress(Exception):
        sock.shutdown(socket.SHUT_RDWR)
    with suppress(Exception):
        sock.close()


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
    """Decode a Draco payload into PLY bytes, using unique filenames per job."""
    ensure_directory(out_dir)
    unique = f"{stem}_{seq}"
    drc_path = out_dir / f"{unique}.drc"
    ply_path = out_dir / f"{unique}.decoded.ply"
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
    ap.add_argument('--decoder-workers', type=int, default=0,
                    help="Parallel decoder workers (0 = auto)")
    return ap


def main(argv: Optional[list[str]] = None) -> None:
    args = build_arg_parser().parse_args(argv)

    decoder = resolve_executable('draco_decoder', args.decoder, env_var='DRACO_DECODER')
    work_dir = ensure_directory(Path(args.work_dir).resolve())

    plan = recommended_worker_count(
        requested=args.decoder_workers,
        reserve_ratio=0.2,
        min_reserve=1,
        cap=None,
        env_prefix="DRACO_DECODER",
    )
    worker_count = plan.workers
    monitor_event(
        "config_workers",
        component="decoder",
        workers=plan.workers,
        mode=plan.mode,
        total_cpu=plan.total_cpu,
        reserved=plan.reserved,
        cap=plan.cap,
    )

    decode_queue: "Queue[Optional[DecodeJob]]" = Queue()
    result_queue: "Queue[Optional[DecodeResult]]" = Queue()
    stop_event = Event()
    bytes_lock = Lock()
    bytes_in = 0
    bytes_out = 0
    job_slots = BoundedSemaphore(worker_count * 2)

    def worker_loop() -> None:
        while True:
            job = decode_queue.get()
            if job is None:
                decode_queue.task_done()
                break
            try:
                monitor_event(
                    "worker_start",
                    seq=job.seq,
                    stem=job.stem,
                    decode_queue=decode_queue.qsize(),
                    result_queue=result_queue.qsize(),
                )
                ply_bytes = decode_drc(decoder, job.payload, work_dir, job.stem, job.seq)
                result_queue.put(DecodeResult(seq=job.seq, stem=job.stem, data=ply_bytes, error=None))
                monitor_event(
                    "worker_done",
                    seq=job.seq,
                    bytes=len(ply_bytes),
                    result_queue=result_queue.qsize(),
                )
            except Exception as exc:  # pylint: disable=broad-except
                result_queue.put(DecodeResult(seq=job.seq, stem=job.stem, data=None, error=str(exc)))
                monitor_event(
                    "worker_fail",
                    seq=job.seq,
                    stem=job.stem,
                    error=str(exc),
                )
            finally:
                job_slots.release()
                decode_queue.task_done()

    workers = [Thread(target=worker_loop, name=f"decoder-{i}", daemon=True) for i in range(worker_count)]
    for t in workers:
        t.start()

    start_time = time.monotonic()

    with socket.create_server((args.host, args.port), reuse_port=True) as server:
        print(f"[SERVER] Listening on {args.host}:{args.port}")
        conn, addr = server.accept()
        print(f"[SERVER] Connection from {addr}")
        conn.settimeout(RECV_TIMEOUT_SEC)

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
                monitor_event(
                    "sender_pending",
                    seq=item.seq,
                    pending=len(pending),
                    next_seq=next_seq,
                    result_queue=result_queue.qsize(),
                )
                while next_seq in pending:
                    result = pending.pop(next_seq)
                    if result.error is not None:
                        msg = Message(kind=MSG_ERROR, name=result.stem, payload=result.error.encode())
                    else:
                        payload = result.data or b""
                        msg = Message(kind=MSG_DATA, name=f"{result.stem}.decoded.ply", payload=payload)
                        with bytes_lock:
                            bytes_out += len(payload)
                    try:
                        send_message(conn, msg)
                    except Exception as exc:  # pylint: disable=broad-except
                        print(f"[SERVER] ERROR sending {result.stem}: {exc}")
                        stop_event.set()
                        shutdown_socket(conn)
                        break
                    next_seq += 1
                    processed += 1
                    monitor_event(
                        "sender_sent",
                        seq=result.seq,
                        pending=len(pending),
                        processed=processed,
                        total_jobs=total_jobs,
                    )
                result_queue.task_done()
                if processed >= total_jobs and decode_queue.empty() and not pending:
                    break

        with conn:
            total_jobs = 0
            sender_thread = Thread(target=sender_loop, daemon=True)
            sender_thread.start()
            seq = 0
            try:
                while not stop_event.is_set():
                    try:
                        msg = recv_message(conn)
                    except socket.timeout:
                        continue
                    except (ConnectionError, OSError) as exc:
                        if not stop_event.is_set():
                            print(f"[SERVER] Connection error: {exc}")
                            stop_event.set()
                        break
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
                    monitor_event(
                        "recv_message",
                        seq=seq,
                        stem=stem,
                        decode_queue=decode_queue.qsize(),
                        total_jobs=total_jobs,
                        payload_bytes=len(msg.payload),
                    )
                    acquired = False
                    while not stop_event.is_set():
                        if job_slots.acquire(timeout=0.1):
                            acquired = True
                            monitor_event(
                                "recv_slot_acquired",
                                seq=seq,
                                pending_decode=decode_queue.qsize(),
                            )
                            break
                    if not acquired:
                        monitor_event(
                            "recv_slot_timeout",
                            seq=seq,
                            pending_decode=decode_queue.qsize(),
                        )
                        break
                    try:
                        decode_queue.put(DecodeJob(seq=seq, stem=stem, payload=msg.payload))
                        monitor_event(
                            "recv_queued",
                            seq=seq,
                            decode_queue=decode_queue.qsize(),
                        )
                    except Exception:
                        job_slots.release()
                        raise
                    seq += 1
                    total_jobs += 1
            finally:
                stop_event.set()
                shutdown_socket(conn)
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
