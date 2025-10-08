#!/usr/bin/env python3
"""Publish a sequence of decoded PLY files as PointCloud2 for RViz preview."""

import argparse
import itertools
from pathlib import Path
from typing import Iterable, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

# Optional Open3D fast path
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


class PlyReplay(Node):
    def __init__(self, files: List[Path], topic: str, frame_id: str, hz: float, loop: bool):
        super().__init__("ply_replay")
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.files = files
        self.loop = loop
        self.frame_id = frame_id
        self.iterator = itertools.cycle(self.files) if loop else iter(self.files)
        period = 1.0 / hz if hz > 0 else 0.0
        self.timer = self.create_timer(max(period, 1e-3), self._tick)

    def _tick(self) -> None:
        try:
            path = next(self.iterator)
        except StopIteration:
            self.get_logger().info("Playback finished")
            rclpy.shutdown()
            return
        pts = _load_xyz(path)
        if pts.size == 0:
            self.get_logger().warning(f"{path.name} has no points; skipping")
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg = pc2.create_cloud_xyz32(header, pts)
        self.pub.publish(msg)
        self.get_logger().info(f"Published {path.name} ({pts.shape[0]} pts)")


def main(argv: Iterable[str] = None) -> None:
    ap = argparse.ArgumentParser(description="Replay decoded PLY files as PointCloud2")
    ap.add_argument("--dir", default="data/tmp_decoded_ply", help="PLY directory")
    ap.add_argument("--prefix", default="sample2", help="filename prefix filter")
    ap.add_argument("--suffix", default=".decoded.ply", help="filename suffix filter")
    ap.add_argument("--topic", default="/replay/pointcloud", help="publish topic")
    ap.add_argument("--frame-id", default="map", help="header frame_id")
    ap.add_argument("--hz", type=float, default=10.0, help="publish rate (Hz)")
    ap.add_argument("--loop", action="store_true", help="loop playback")
    args = ap.parse_args(argv)

    root = Path.cwd()
    ply_dir = (root / args.dir).resolve()
    files = sorted(p for p in ply_dir.glob(f"{args.prefix}_*{args.suffix}"))
    if not files:
        raise SystemExit(f"No files matching prefix '{args.prefix}' in {ply_dir}")

    rclpy.init()
    node = PlyReplay(files, args.topic, args.frame_id, args.hz, args.loop)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
