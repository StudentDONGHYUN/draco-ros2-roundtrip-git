#!/usr/bin/env python3
"""Replay original & decoded PLY pairs as PointCloud2 topics for RViz comparison."""

import argparse
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

try:
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:  # pragma: no cover - optional dependency
    _HAVE_O3D = False

from plyfile import PlyData


def _load_xyz(path: Path) -> np.ndarray:
    if _HAVE_O3D:
        pcd = o3d.io.read_point_cloud(str(path))
        return np.asarray(pcd.points, dtype=np.float32)
    ply = PlyData.read(str(path))
    verts = ply["vertex"]
    arr = np.vstack((verts["x"], verts["y"], verts["z"]))
    return arr.T.astype(np.float32)


def _trim_suffix(name: str, suffix: str) -> str:
    return name[:-len(suffix)] if suffix and name.endswith(suffix) else name


def _collect_pairs(orig_dir: Path, dec_dir: Path, prefix: str,
                   suffix_orig: str, suffix_dec: str) -> List[Tuple[Path, Path, str]]:
    orig_files = { _trim_suffix(p.name, suffix_orig): p for p in orig_dir.glob(f"{prefix}_*{suffix_orig}") }
    dec_files = { _trim_suffix(p.name, suffix_dec): p for p in dec_dir.glob(f"{prefix}_*{suffix_dec}") }

    stems = sorted(set(orig_files.keys()) & set(dec_files.keys()),
                   key=lambda s: int(s.split("_")[-1]) if s.split("_")[-1].isdigit() else s)
    pairs: List[Tuple[Path, Path, str]] = []
    for stem in stems:
        pairs.append((orig_files[stem], dec_files[stem], stem))
    return pairs


class PairReplay(Node):
    def __init__(self, pairs: List[Tuple[Path, Path, str]], frame_id: str,
                 topic_prefix: str, hz: float, loop: bool):
        super().__init__("ply_pair_replay")

        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub_src = self.create_publisher(PointCloud2, f"/{topic_prefix}/source", qos)
        self.pub_dec = self.create_publisher(PointCloud2, f"/{topic_prefix}/decoded", qos)

        self.frame_id = frame_id
        self.pairs = pairs
        self.loop = loop
        self.index = 0
        self.period = 1.0 / hz if hz > 0 else 0.0
        self.timer = self.create_timer(max(self.period, 1e-3), self._tick)

    def _tick(self) -> None:
        if self.index >= len(self.pairs):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info("Playback complete; shutting down.")
                rclpy.shutdown()
                return

        orig, dec, stem = self.pairs[self.index]
        self.index += 1

        xyz_src = _load_xyz(orig)
        xyz_dec = _load_xyz(dec)
        if xyz_src.size == 0 and xyz_dec.size == 0:
            self.get_logger().warning(f"{stem}: both point clouds empty; skipping")
            return

        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        if xyz_src.size:
            msg_src = pc2.create_cloud_xyz32(header, xyz_src)
            self.pub_src.publish(msg_src)
        if xyz_dec.size:
            msg_dec = pc2.create_cloud_xyz32(header, xyz_dec)
            self.pub_dec.publish(msg_dec)

        self.get_logger().info(
            f"Published {stem}: source={xyz_src.shape[0]} pts decoded={xyz_dec.shape[0]} pts"
        )


def main(argv: Iterable[str] = None) -> None:
    ap = argparse.ArgumentParser(description="Replay original/decoded PLY pairs for RViz")
    ap.add_argument("--orig-dir", default="data/ply_raw", help="원본 PLY 디렉토리")
    ap.add_argument("--decoded-dir", default="data/tmp_decoded_ply", help="디코드 PLY 디렉토리")
    ap.add_argument("--prefix", default="sample2", help="파일 접두어")
    ap.add_argument("--orig-suffix", default=".ply", help="원본 파일 접미사")
    ap.add_argument("--decoded-suffix", default=".decoded.ply", help="디코드 파일 접미사")
    ap.add_argument("--topic-prefix", default="compare", help="퍼블리시 토픽 접두어")
    ap.add_argument("--frame-id", default="map", help="header frame_id")
    ap.add_argument("--hz", type=float, default=5.0, help="재생 속도(Hz)")
    ap.add_argument("--loop", action="store_true", help="끝나면 처음부터 반복")
    ap.add_argument("--limit", type=int, default=0, help="0=전체, 양수=앞에서 N개만")
    args = ap.parse_args(argv)

    orig_dir = Path(args.orig_dir).expanduser().resolve()
    dec_dir = Path(args.decoded_dir).expanduser().resolve()

    pairs = _collect_pairs(orig_dir, dec_dir, args.prefix,
                           args.orig_suffix, args.decoded_suffix)
    if args.limit > 0:
        pairs = pairs[: args.limit]
    if not pairs:
        raise SystemExit("No matching pairs found")

    rclpy.init()
    try:
        node = PairReplay(pairs, args.frame_id, args.topic_prefix, args.hz, args.loop)
        rclpy.spin(node)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
