"""Metric helpers used by legacy draco_roundtrip tools."""

from __future__ import annotations

import numpy as np

try:  # Optional dependency: SciPy accelerates Chamfer calculations
    from scipy.spatial import cKDTree  # type: ignore
    _HAVE_SCIPY = True
except Exception:  # pragma: no cover - SciPy may be unavailable on target devices
    _HAVE_SCIPY = False

__all__ = ["_HAVE_SCIPY", "sample_indices", "compute_basic_metrics"]


def sample_indices(length: int, sample: int) -> np.ndarray:
    """Return evenly spaced indices for sampling ``length`` points."""
    if sample <= 0 or sample >= length:
        return np.arange(length)
    step = max(length // sample, 1)
    return np.arange(0, length, step)[:sample]


def compute_basic_metrics(src_pts: np.ndarray, dec_pts: np.ndarray, sample: int) -> dict[str, object]:
    """Compute basic comparison metrics between source and decoded point clouds."""
    n_src = len(src_pts)
    n_dec = len(dec_pts)
    diff = n_dec - n_src

    centroid_src = src_pts.mean(axis=0) if n_src else np.zeros(3, dtype=np.float32)
    centroid_dec = dec_pts.mean(axis=0) if n_dec else np.zeros(3, dtype=np.float32)
    centroid_delta = centroid_dec - centroid_src
    centroid_norm = float(np.linalg.norm(centroid_delta))

    bbox_src_min = src_pts.min(axis=0) if n_src else np.zeros(3, dtype=np.float32)
    bbox_src_max = src_pts.max(axis=0) if n_src else np.zeros(3, dtype=np.float32)
    bbox_dec_min = dec_pts.min(axis=0) if n_dec else np.zeros(3, dtype=np.float32)
    bbox_dec_max = dec_pts.max(axis=0) if n_dec else np.zeros(3, dtype=np.float32)
    bbox_delta = (bbox_dec_max - bbox_dec_min) - (bbox_src_max - bbox_src_min)

    chamfer_mean: str | float = "n/a"
    chamfer_max: str | float = "n/a"

    if n_src and n_dec:
        idx_src = sample_indices(n_src, sample if sample > 0 else n_src)
        idx_dec = sample_indices(n_dec, sample if sample > 0 else n_dec)
        pts_src = src_pts[idx_src]
        pts_dec = dec_pts[idx_dec]
        if _HAVE_SCIPY and pts_src.size and pts_dec.size:
            kd_dec = cKDTree(pts_dec)
            kd_src = cKDTree(pts_src)
            d_src = kd_dec.query(pts_src, k=1)[0]
            d_dec = kd_src.query(pts_dec, k=1)[0]
            dists = np.concatenate([d_src, d_dec])
            if len(dists):
                chamfer_mean = f"{float(np.mean(dists)):.3f}"
                chamfer_max = f"{float(np.max(dists)):.3f}"
        else:
            m = min(len(pts_src), len(pts_dec))
            if m:
                diffs = pts_dec[:m] - pts_src[:m]
                norms = np.linalg.norm(diffs, axis=1)
                chamfer_mean = f"{float(np.mean(norms)):.3f}"
                chamfer_max = f"{float(np.max(norms)):.3f}"

    return {
        "n_src": n_src,
        "n_dec": n_dec,
        "diff": diff,
        "centroid_delta": centroid_delta,
        "centroid_norm": centroid_norm,
        "bbox_delta": bbox_delta,
        "chamfer_mean": chamfer_mean,
        "chamfer_max": chamfer_max,
    }
