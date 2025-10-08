"""Configuration helpers for locating resources used by draco_roundtrip."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

try:  # Optional dependency during build or when running outside ROS
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - CLI tools might run without ament
    get_package_share_directory = None  # type: ignore[assignment]

__all__ = ["ensure_directory", "resolve_qos_override"]


def ensure_directory(path: Path) -> Path:
    """Create ``path`` if needed and return it."""
    path.mkdir(parents=True, exist_ok=True)
    return path


def resolve_qos_override(package: str = "draco_roundtrip") -> Optional[Path]:
    """Return the first existing QoS override YAML shipped with the package."""
    candidates: list[Path] = []
    if get_package_share_directory:
        try:
            share_dir = Path(get_package_share_directory(package))
            candidates.append(share_dir / "config" / "qos_override.yaml")
            candidates.append(share_dir / "configs" / "qos_override.yaml")
        except Exception:
            pass

    here = Path(__file__).resolve()
    candidates.append(here.parents[2] / "configs" / "qos_override.yaml")
    candidates.append(here.parents[3] / "configs" / "qos_override.yaml")

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None
