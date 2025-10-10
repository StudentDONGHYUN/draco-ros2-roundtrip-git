"""Streaming pipeline telemetry monitor.

모니터링 프로그램은 서버/클라이언트가 보내는 JSON 텔레메트리를 수신해
파이프라인 진행 상황과 병목 가능성을 분석한다. 모니터를 먼저 띄운 뒤
`ros2 run draco_roundtrip stream_server`, `stream_client`를 실행하면 된다.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import signal
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Iterable, Optional, Tuple


@dataclass
class StageState:
    seq: int = -1
    updated: float = 0.0


PIPELINE_FLOW = [
    ("client", "reader_enqueued"),
    ("client", "sender_sent"),
    ("server", "recv_queued"),
    ("server", "sender_sent"),
    ("client", "receiver_data"),
]


class TelemetryState:
    def __init__(self, stall_gap: int, stall_seconds: float) -> None:
        self.stage_state: Dict[Tuple[str, str], StageState] = {
            key: StageState() for key in PIPELINE_FLOW
        }
        self.stall_gap = stall_gap
        self.stall_seconds = stall_seconds
        self._last_event = time.monotonic()
        self._warning_last: Dict[Tuple[str, str], float] = defaultdict(float)
        self._activity = False

    def process_event(self, event: Dict[str, object]) -> None:
        self._last_event = time.monotonic()
        self._activity = True
        role = str(event.get("role", "unknown"))
        name = str(event.get("event", "unknown"))
        seq = event.get("seq")
        if isinstance(seq, bool):
            seq = None
        if isinstance(seq, (int, float)):
            stage = self.stage_state.get((role, name))
            if stage is not None and seq > stage.seq:
                stage.seq = int(seq)
                stage.updated = self._last_event

    def check_stalls(self) -> Iterable[str]:
        if not self._activity:
            return ()
        now = time.monotonic()
        for idx in range(len(PIPELINE_FLOW) - 1):
            a_key = PIPELINE_FLOW[idx]
            b_key = PIPELINE_FLOW[idx + 1]
            a_state = self.stage_state[a_key]
            b_state = self.stage_state[b_key]
            if a_state.seq < 0 or b_state.seq < 0:
                continue
            gap = a_state.seq - b_state.seq
            if gap <= self.stall_gap:
                continue
            stagnation = now - min(a_state.updated, b_state.updated)
            if stagnation < self.stall_seconds:
                continue
            warn_key = (a_key[1], b_key[1])
            last_warn = self._warning_last[warn_key]
            if now - last_warn >= self.stall_seconds:
                self._warning_last[warn_key] = now
                role_a, stage_a = a_key
                role_b, stage_b = b_key
                yield (
                    f"Potential stall: {role_a}.{stage_a} seq={a_state.seq} "
                    f"is ahead of {role_b}.{stage_b} seq={b_state.seq} by {gap} frames "
                    f"(no progress for {stagnation:.1f}s)"
                )

    def summary_line(self) -> str:
        parts = []
        for role, stage in PIPELINE_FLOW:
            state = self.stage_state[(role, stage)]
            seq = "-" if state.seq < 0 else str(state.seq)
            parts.append(f"{role}.{stage}={seq}")
        return " | ".join(parts)

    def has_activity(self) -> bool:
        return self._activity


class MonitorServer:
    def __init__(self, host: str, port: int, stall_gap: int,
                 stall_seconds: float, summary_interval: float) -> None:
        self.host = host
        self.port = port
        self.state = TelemetryState(stall_gap=stall_gap, stall_seconds=stall_seconds)
        self.summary_interval = summary_interval
        self._server: Optional[asyncio.base_events.Server] = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(self.handle_client, self.host, self.port)
        addr = ", ".join(str(sock.getsockname()) for sock in self._server.sockets or [])
        print(f"[MONITOR] Listening on {addr}")
        asyncio.create_task(self._summary_loop())
        asyncio.create_task(self._stall_loop())

    async def wait_closed(self) -> None:
        if self._server is None:
            return
        async with self._server:
            await self._server.serve_forever()

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        print(f"[MONITOR] Connection from {peer}")
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    print(f"[MONITOR] WARN: invalid JSON from {peer}: {line!r}")
                    continue
                self._log_event(event)
                self.state.process_event(event)
        except asyncio.IncompleteReadError:
            pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:  # pragma: no cover - defensive
                pass
            print(f"[MONITOR] Disconnected: {peer}")

    def _log_event(self, event: Dict[str, object]) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        role = event.get("role", "unknown")
        name = event.get("event", "unknown")
        extras = []
        for key, value in sorted(event.items()):
            if key in {"ts", "role", "event"}:
                continue
            extras.append(f"{key}={value}")
        tail = " ".join(extras)
        if tail:
            print(f"[{ts}] {role}.{name} {tail}")
        else:
            print(f"[{ts}] {role}.{name}")

    async def _summary_loop(self) -> None:
        while True:
            await asyncio.sleep(self.summary_interval)
            if not self.state.has_activity():
                continue
            print(f"[SUMMARY] {self.state.summary_line()}")

    async def _stall_loop(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            if not self.state.has_activity():
                continue
            for warning in self.state.check_stalls():
                print(f"[WARN] {warning}")


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Draco stream telemetry monitor")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=55050)
    ap.add_argument("--stall-gap", type=int, default=3,
                    help="Allowed frame difference between consecutive stages")
    ap.add_argument("--stall-seconds", type=float, default=5.0,
                    help="Warning threshold for stalled progress")
    ap.add_argument("--summary-interval", type=float, default=5.0,
                    help="Status summary print interval (seconds)")
    return ap


def main(argv: Iterable[str] | None = None) -> None:
    args = build_arg_parser().parse_args(argv)

    async def _run() -> None:
        monitor = MonitorServer(
            host=args.host,
            port=args.port,
            stall_gap=args.stall_gap,
            stall_seconds=args.stall_seconds,
            summary_interval=args.summary_interval,
        )
        await monitor.start()
        await monitor.wait_closed()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, loop.stop)
        except NotImplementedError:  # Windows
            signal.signal(sig, lambda _sig, _frame: loop.stop())

    try:
        loop.run_until_complete(_run())
    except KeyboardInterrupt:
        pass
    finally:
        loop.stop()
        loop.close()
        print("[MONITOR] Shutdown complete")


if __name__ == "__main__":
    main()
