"""Lightweight monitoring client for streaming telemetry."""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

DEFAULT_ADDR = "127.0.0.1:55050"
RETRY_INTERVAL_SEC = 5.0
CONNECT_TIMEOUT_SEC = 0.5


def _parse_addr(addr: str) -> Tuple[str, int]:
    host, _, port = addr.partition(":")
    if not host:
        host = "127.0.0.1"
    if not port:
        raise ValueError(f"Invalid monitor address {addr!r}")
    return host, int(port)


@dataclass
class _SocketState:
    sock: Optional[socket.socket]
    next_retry: float


class MonitorClient:
    """Send structured telemetry events to an optional local monitor process.

    The client is designed to be non-intrusive: if the monitor is not running
    or the connection drops, events are silently ignored after a quick retry.
    """

    def __init__(self, role: str, addr: Optional[str] = None) -> None:
        self.role = role
        self._addr = _parse_addr(addr or os.environ.get("DRACO_MONITOR_ADDR", DEFAULT_ADDR))
        self._state = _SocketState(sock=None, next_retry=0.0)
        self._lock = threading.Lock()

    def emit(self, event: str, **payload: Any) -> None:
        """Emit a telemetry event.

        Events are newline-delimited JSON objects with at minimum:
        - ``ts``: wall-clock time
        - ``role``: high-level process identifier (client/server)
        - ``event``: short event name (e.g., reader_enqueued)

        Additional keyword arguments provide event-specific metrics.
        """
        now = time.monotonic()
        if self._state.sock is None and now >= self._state.next_retry:
            self._attempt_connect()
        if self._state.sock is None:
            return

        envelope: Dict[str, Any] = {
            "ts": time.time(),
            "role": self.role,
            "event": event,
            **payload,
        }
        data = json.dumps(envelope, separators=(",", ":")).encode("utf-8") + b"\n"

        with self._lock:
            sock = self._state.sock
            if sock is None:
                return
            try:
                sock.sendall(data)
            except OSError:
                try:
                    sock.close()
                except OSError:
                    pass
                self._state = _SocketState(sock=None, next_retry=now + RETRY_INTERVAL_SEC)

    def close(self) -> None:
        with self._lock:
            sock = self._state.sock
            self._state = _SocketState(sock=None, next_retry=float("inf"))
        if sock:
            try:
                sock.close()
            except OSError:
                pass

    def _attempt_connect(self) -> None:
        try:
            sock = socket.create_connection(self._addr, timeout=CONNECT_TIMEOUT_SEC)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._state = _SocketState(sock=sock, next_retry=0.0)
        except OSError:
            self._state = _SocketState(sock=None, next_retry=time.monotonic() + RETRY_INTERVAL_SEC)


def create_monitor(role: str) -> MonitorClient:
    """Factory helper that returns a shared monitor client for a given role."""
    client = MonitorClient(role)
    return client
