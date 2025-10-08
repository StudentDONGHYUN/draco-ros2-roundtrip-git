#!/usr/bin/env python3
"""Publish original vs decoded PLY pair for RViz overlay."""

import argparse
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

try:
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:
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


class PairPublisher(Node):
    def __init__(self, src: Path, dec: Path, frame_id: str, hz: float, prefix: str):
        super().__init__("ply_pair_publisher")
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub_src = self.create_publisher(PointCloud2, f"/{prefix}/source", qos)
        self.pub_dec = self.create_publisher(PointCloud2, f"/{prefix}/decoded", qos)
        self.frame_id = frame_id
        self.src_pts = _load_xyz(src)
        self.dec_pts = _load_xyz(dec)
        if self.src_pts.size == 0 or self.dec_pts.size == 0:
            raise RuntimeError("One of the inputs is empty")
        self.period = 1.0 / hz if hz > 0 else 0.0
        self.timer = self.create_timer(max(self.period, 1e-3), self._tick)
        self.get_logger().info(
            f"Loaded source={src.name} ({self.src_pts.shape[0]} pts), decoded={dec.name} ({self.dec_pts.shape[0]} pts)"
        )

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        src_msg = pc2.create_cloud_xyz32(header, self.src_pts)
        dec_msg = pc2.create_cloud_xyz32(header, self.dec_pts)
        self.pub_src.publish(src_msg)
        self.pub_dec.publish(dec_msg)


def main(argv: Iterable[str] = None) -> None:
    ap = argparse.ArgumentParser(description="Publish a PLY pair (source vs decoded)")
    ap.add_argument("--src", required=True, help="source/original PLY path")
    ap.add_argument("--decoded", required=True, help="decoded PLY path")
    ap.add_argument("--frame-id", default="map")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--topic-prefix", default="ply_compare")
    args = ap.parse_args(argv)

    src = Path(args.src).expanduser().resolve()
    dec = Path(args.decoded).expanduser().resolve()
    if not src.exists() or not dec.exists():
        raise SystemExit("Input files not found")

    rclpy.init()
    try:
        node = PairPublisher(src, dec, args.frame_id, args.hz, args.topic_prefix)
        rclpy.spin(node)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
