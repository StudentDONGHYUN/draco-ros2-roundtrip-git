"""PLY and PointCloud helpers maintained for backwards compatibility."""

from __future__ import annotations

import io
import re
from pathlib import Path

import numpy as np

try:  # Optional dependency: Open3D speeds up I/O
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:  # pragma: no cover - Open3D is optional
    _HAVE_O3D = False

from plyfile import PlyData, PlyElement  # type: ignore

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

__all__ = [
    "_HAVE_O3D",
    "collect_matching_pairs",
    "ensure_suffix",
    "load_points",
    "load_points_from_bytes",
    "points_from_pointcloud2",
    "voxel_downsample",
    "save_points",
]


def load_points(path: Path) -> np.ndarray:
    """Load XYZ points from ``path``."""
    if _HAVE_O3D:
        pcd = o3d.io.read_point_cloud(str(path))
        return np.asarray(pcd.points, dtype=np.float32)
    ply = PlyData.read(str(path))
    verts = ply["vertex"]
    arr = np.vstack((verts["x"], verts["y"], verts["z"]))
    return arr.T.astype(np.float32)


def load_points_from_bytes(data: bytes) -> np.ndarray:
    ply = PlyData.read(io.BytesIO(data))
    verts = ply["vertex"]
    arr = np.vstack((verts["x"], verts["y"], verts["z"]))
    return arr.T.astype(np.float32)


def points_from_pointcloud2(msg: PointCloud2) -> np.ndarray:
    """Convert a PointCloud2 message into an (N, 3) float32 array."""
    try:
        arr = pc2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim == 1:
            if arr.size == 0:
                return np.empty((0, 3), dtype=np.float32)
            arr = arr.reshape((-1, 3))
        elif arr.shape[1] > 3:
            arr = arr[:, :3]
        return arr
    except Exception:
        pass

    xyz_list = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        try:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
        except Exception:
            x = float(p["x"])
            y = float(p["y"])
            z = float(p["z"])
        xyz_list.append((x, y, z))
    if not xyz_list:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(xyz_list, dtype=np.float32)


def voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample points using a voxel grid."""
    if voxel_size <= 0.0 or xyz.size == 0:
        return xyz
    if _HAVE_O3D:
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
        ds = pcd.voxel_down_sample(voxel_size)
        return np.asarray(ds.points, dtype=np.float32)
    scaled = np.floor(xyz / voxel_size)
    structured = np.core.records.fromarrays(scaled.T, names="x,y,z", formats="i8,i8,i8")
    _, idx = np.unique(structured, return_index=True)
    return xyz[idx]


def save_points(path: Path, xyz_f32: np.ndarray) -> None:
    """Write XYZ points to a binary PLY file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not _save_ply_o3d(path, xyz_f32):
        _save_ply_plyfile(path, xyz_f32)


def _save_ply_plyfile(path: Path, xyz_f32: np.ndarray) -> None:
    verts = np.zeros(xyz_f32.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    verts["x"] = xyz_f32[:, 0]
    verts["y"] = xyz_f32[:, 1]
    verts["z"] = xyz_f32[:, 2]
    el = PlyElement.describe(verts, "vertex")
    PlyData([el], text=False).write(str(path))


def _save_ply_o3d(path: Path, xyz_f32: np.ndarray) -> bool:
    if not _HAVE_O3D:
        return False
    try:
        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3d.core.Tensor(xyz_f32.astype(np.float32), dtype=o3d.core.Dtype.Float32)
        o3d.t.io.write_point_cloud(str(path), pcd, write_ascii=False)
        with open(path, "rb") as f:
            head = f.read(512).decode("utf-8", "ignore")
        if re.search(r"property\s+float\s+x", head) and \
           re.search(r"property\s+float\s+y", head) and \
           re.search(r"property\s+float\s+z", head):
            return True
    except Exception:
        return False
    return False


def ensure_suffix(name: str, suffix: str) -> str:
    """Ensure ``name`` ends with ``suffix``."""
    return name if name.endswith(suffix) else f"{name}{suffix}"


def collect_matching_pairs(orig_dir: Path, dec_dir: Path, prefix: str,
                           orig_suffix: str, dec_suffix: str) -> list[tuple[Path, Path, str]]:
    """Return matching PLY pairs ordered by sequence number."""
    def _trim(path: Path, suffix: str) -> str:
        if suffix and path.name.endswith(suffix):
            return path.name[:-len(suffix)]
        return path.stem

    orig_map = {_trim(p, orig_suffix): p for p in orig_dir.glob(f"{prefix}_*{orig_suffix}")}
    dec_map = {_trim(p, dec_suffix): p for p in dec_dir.glob(f"{prefix}_*{dec_suffix}")}
    keys = sorted(orig_map.keys() & dec_map.keys(), key=_key_order)
    return [(orig_map[k], dec_map[k], k) for k in keys]


def _key_order(name: str) -> tuple[int, str]:
    parts = name.split("_")
    tail = parts[-1]
    if tail.isdigit():
        return (0, int(tail))
    return (1, name)
