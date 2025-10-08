"""TCP protocol helpers shared by client/server."""

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from typing import Optional

__all__ = [
    "Message",
    "ProtocolError",
    "ConnectionClosed",
    "MSG_DATA",
    "MSG_ERROR",
    "MSG_EOF",
    "send_message",
    "recv_message",
]

_HEADER = struct.Struct("!I")  # name length
_SIZE = struct.Struct("!Q")  # payload length
_SEPARATOR = ":"

MSG_DATA = "data"
MSG_ERROR = "error"
MSG_EOF = "eof"


class ProtocolError(RuntimeError):
    """Raised when the TCP framing is malformed."""


class ConnectionClosed(RuntimeError):
    """Raised when the peer closes the connection unexpectedly."""


@dataclass(slots=True)
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


def _read_exact(sock: socket.socket, size: int) -> bytes:
    buf = bytearray()
    while len(buf) < size:
        chunk = sock.recv(size - len(buf))
        if not chunk:
            raise ConnectionClosed("socket closed while reading")
        buf.extend(chunk)
    return bytes(buf)


def send_message(sock: socket.socket, message: Message) -> None:
    meta = message.as_meta().encode("utf-8")
    sock.sendall(_HEADER.pack(len(meta)))
    sock.sendall(meta)
    sock.sendall(_SIZE.pack(len(message.payload)))
    if message.payload:
        sock.sendall(message.payload)


def recv_message(sock: socket.socket) -> Optional[Message]:
    header = sock.recv(_HEADER.size)
    if not header:
        return None
    if len(header) != _HEADER.size:
        raise ProtocolError("incomplete header")
    (name_len,) = _HEADER.unpack(header)
    if name_len <= 0:
        return None
    meta = _read_exact(sock, name_len).decode("utf-8")
    (payload_len,) = _SIZE.unpack(_read_exact(sock, _SIZE.size))
    payload = _read_exact(sock, payload_len) if payload_len else b""
    return Message.from_meta(meta, payload)
