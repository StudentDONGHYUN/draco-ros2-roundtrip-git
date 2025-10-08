"""Legacy compatibility layer exposing commonly used helpers."""

from .config import ensure_directory, resolve_qos_override
from .executable import resolve_executable
from .metrics import _HAVE_SCIPY, compute_basic_metrics, sample_indices
from .ply_io import (
    _HAVE_O3D,
    collect_matching_pairs,
    ensure_suffix,
    load_points,
    load_points_from_bytes,
    points_from_pointcloud2,
    save_points,
    voxel_downsample,
)
from .protocol import (
    ConnectionClosed,
    Message,
    ProtocolError,
    MSG_DATA,
    MSG_EOF,
    MSG_ERROR,
    recv_message,
    send_message,
)

__all__ = [
    "ensure_directory",
    "resolve_qos_override",
    "resolve_executable",
    "compute_basic_metrics",
    "sample_indices",
    "collect_matching_pairs",
    "ensure_suffix",
    "load_points",
    "load_points_from_bytes",
    "points_from_pointcloud2",
    "save_points",
    "voxel_downsample",
    "_HAVE_O3D",
    "_HAVE_SCIPY",
    "ConnectionClosed",
    "ProtocolError",
    "Message",
    "MSG_DATA",
    "MSG_EOF",
    "MSG_ERROR",
    "send_message",
    "recv_message",
]
