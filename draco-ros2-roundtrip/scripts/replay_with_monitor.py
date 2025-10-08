#!/usr/bin/env python3
"""Replay PLY pairs and emit per-frame diff metrics while publishing to ROS."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

try:
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:  # pragma: no cover - optional
    _HAVE_O3D = False

try:
    from scipy.spatial import cKDTree
    _HAVE_SCIPY = True
except Exception:  # pragma: no cover - optional
    _HAVE_SCIPY = False

from plyfile import PlyData


def _load_xyz(path: Path) -> np.ndarray:
    if _HAVE_O3D:
        pcd = o3d.io.read_point_cloud(str(path))
        pts = np.asarray(pcd.points, dtype=np.float32)
        return pts
    ply = PlyData.read(str(path))
    verts = ply["vertex"]
    arr = np.vstack((verts["x"], verts["y"], verts["z"]))
    return arr.T.astype(np.float32)


def _trim_suffix(name: str, suffix: str) -> str:
    return name[:-len(suffix)] if suffix and name.endswith(suffix) else name


def collect_pairs(orig_dir: Path, dec_dir: Path, prefix: str,
                  orig_suffix: str, dec_suffix: str) -> List[Tuple[Path, Path, str]]:
    def norm_name(path: Path, suffix: str) -> str:
        name = path.name
        if suffix and name.endswith(suffix):
            return name[:-len(suffix)]
        return path.stem

    orig_map = {}
    for p in orig_dir.glob(f"{prefix}_*{orig_suffix}"):
        key = norm_name(p, orig_suffix)
        orig_map[key] = p

    dec_map = {}
    for p in dec_dir.glob(f"{prefix}_*{dec_suffix}"):
        key = norm_name(p, dec_suffix)
        dec_map[key] = p

    common_keys = sorted(orig_map.keys() & dec_map.keys(), key=lambda s: int(s.split('_')[-1]) if s.split('_')[-1].isdigit() else s)
    return [(orig_map[k], dec_map[k], k) for k in common_keys]


def sample_indices(length: int, sample: int) -> np.ndarray:
    if sample <= 0 or sample >= length:
        return np.arange(length)
    step = max(length // sample, 1)
    return np.arange(0, length, step)[:sample]


def compute_diffs(src_pts: np.ndarray, dec_pts: np.ndarray,
                  sample: int) -> Tuple[int, int, int, np.ndarray, float, np.ndarray, str, str]:
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

    bbox_size_src = bbox_src_max - bbox_src_min
    bbox_size_dec = bbox_dec_max - bbox_dec_min
    bbox_delta = bbox_size_dec - bbox_size_src

    mean_str = "n/a"
    max_str = "n/a"

    if n_src and n_dec:
        m = min(n_src, n_dec)
        if sample > 0:
            m = min(m, sample)
        idx_src = sample_indices(n_src, m)
        idx_dec = sample_indices(n_dec, m)
        pts_src_sample = src_pts[idx_src]
        pts_dec_sample = dec_pts[idx_dec]

        if _HAVE_SCIPY and m > 0:
            kd_dec = cKDTree(pts_dec_sample)
            kd_src = cKDTree(pts_src_sample)
            d_src = kd_dec.query(pts_src_sample, k=1)[0]
            d_dec = kd_src.query(pts_dec_sample, k=1)[0]
            dists = np.concatenate([d_src, d_dec])
            mean_str = f"{float(np.mean(dists)):.3f}"
            max_str = f"{float(np.max(dists)):.3f}"
        else:
            m = min(len(pts_src_sample), len(pts_dec_sample))
            if m > 0:
                diffs = pts_dec_sample[:m] - pts_src_sample[:m]
                norms = np.linalg.norm(diffs, axis=1)
                mean_str = f"{float(np.mean(norms)):.3f}"
                max_str = f"{float(np.max(norms)):.3f}"

    return n_src, n_dec, diff, centroid_delta, centroid_norm, bbox_delta, mean_str, max_str


class PairPublisher(Node):
    def __init__(self, topic_prefix: str):
        super().__init__('ply_pair_replay_with_diff')
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub_src = self.create_publisher(PointCloud2, f"/{topic_prefix}/source", qos)
        self.pub_dec = self.create_publisher(PointCloud2, f"/{topic_prefix}/decoded", qos)

    def publish_pair(self, frame_id: str, pts_src: np.ndarray, pts_dec: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=frame_id)
        msg_src = pc2.create_cloud_xyz32(header, pts_src.astype(np.float32))
        msg_dec = pc2.create_cloud_xyz32(header, pts_dec.astype(np.float32))
        self.pub_src.publish(msg_src)
        self.pub_dec.publish(msg_dec)


def main(argv: Iterable[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Replay PLY pairs while printing per-frame diffs")
    ap.add_argument("--orig-dir", default="data/ply_raw")
    ap.add_argument("--decoded-dir", default="data/tmp_decoded_ply")
    ap.add_argument("--prefix", default="sample2")
    ap.add_argument("--orig-suffix", default=".ply")
    ap.add_argument("--decoded-suffix", default=".decoded.ply")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--frame-id", default="lidar_link")
    ap.add_argument("--topic-prefix", default="sample2_pair")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--sample", type=int, default=50000)

    args = ap.parse_args(argv)

    orig_dir = Path(args.orig_dir).resolve()
    dec_dir = Path(args.decoded_dir).resolve()

    pairs = collect_pairs(orig_dir, dec_dir, args.prefix, args.orig_suffix, args.decoded_suffix)
    if args.limit > 0:
        pairs = pairs[:args.limit]
    if not pairs:
        raise SystemExit("No matching PLY pairs found")

    interval = 1.0 / args.hz if args.hz > 0 else 0.0

    rclpy.init()
    node = PairPublisher(args.topic_prefix)
    try:
        frame = 0
        while True:
            for orig_path, dec_path, stem in pairs:
                frame += 1
                pts_src = _load_xyz(orig_path)
                pts_dec = _load_xyz(dec_path)

                n_src, n_dec, diff, centroid_delta, centroid_norm, bbox_delta, mean_str, max_str = compute_diffs(
                    pts_src, pts_dec, args.sample)

                print(f"[FRAME {frame:04d}] {stem}")
                print(f"  points:  src={n_src}  dec={n_dec}  diff={diff:+d}")
                print(
                    "  centroid diff (m):  "
                    f"x={centroid_delta[0]:+.3f}  y={centroid_delta[1]:+.3f}  z={centroid_delta[2]:+.3f}  ‖Δ‖={centroid_norm:.3f}"
                )
                print(
                    "  bbox size Δ (m):   "
                    f"x={bbox_delta[0]:+.3f}  y={bbox_delta[1]:+.3f}  z={bbox_delta[2]:+.3f}"
                )
                print(f"  mean|max dist (m): {mean_str}/{max_str}")

                node.publish_pair(args.frame_id, pts_src, pts_dec)
                rclpy.spin_once(node, timeout_sec=0.0)
                if interval > 0:
                    time.sleep(interval)
            if not args.loop:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
