"""Locate external executables used by draco_roundtrip tools."""

from __future__ import annotations

import os
from pathlib import Path
from shutil import which
from typing import Optional

__all__ = ["resolve_executable"]


def _candidate_from_hint(name: str, hint: Optional[str]) -> Optional[Path]:
    if not hint:
        return None
    hint_path = Path(hint).expanduser().resolve()
    if hint_path.is_file():
        return hint_path
    candidate = hint_path / name
    if candidate.exists():
        return candidate
    return None


def resolve_executable(name: str, hint: Optional[str] = None, env_var: Optional[str] = None) -> Path:
    """Resolve ``name`` by checking hint, environment variable, PATH, and common fallbacks."""
    path = _candidate_from_hint(name, hint)
    if path:
        return path

    if env_var:
        env_val = os.environ.get(env_var)
        path = _candidate_from_hint(name, env_val) if env_val else None
        if path:
            return path

    found = which(name)
    if found:
        return Path(found).resolve()

    home = Path.home()
    fallbacks = [
        home / "draco" / "build" / "bin" / name,
        home / "draco" / "build" / name,
    ]
    for candidate in fallbacks:
        if candidate.exists():
            return candidate.resolve()

    raise FileNotFoundError(f"Unable to locate required executable: {name}")
