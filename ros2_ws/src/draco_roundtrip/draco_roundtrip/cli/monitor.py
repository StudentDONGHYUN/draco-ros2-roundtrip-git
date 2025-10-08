"""CLI wrapper for the point cloud monitor tool."""

from __future__ import annotations

from draco_roundtrip.tools.monitor import main as tool_main


def main() -> None:
    tool_main()
