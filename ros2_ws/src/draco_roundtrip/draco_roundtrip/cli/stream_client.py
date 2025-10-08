"""CLI wrapper for the streaming client node."""

from __future__ import annotations

from draco_roundtrip.nodes.stream_client import main as node_main


def main() -> None:
    node_main()
