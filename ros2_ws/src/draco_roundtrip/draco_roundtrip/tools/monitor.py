#!/usr/bin/env python3
"""Replay PLY pairs and emit per-frame diff metrics while publishing to ROS."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from draco_roundtrip.utils import (
    collect_matching_pairs,
    compute_basic_metrics,
    load_points,
)


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


def build_arg_parser() -> argparse.ArgumentParser:
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
    return ap


def main(argv: Iterable[str] | None = None) -> None:
    args = build_arg_parser().parse_args(argv)

    orig_dir = Path(args.orig_dir).resolve()
    dec_dir = Path(args.decoded_dir).resolve()

    pairs = collect_matching_pairs(orig_dir, dec_dir, args.prefix,
                                   args.orig_suffix, args.decoded_suffix)
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
                pts_src = load_points(orig_path)
                pts_dec = load_points(dec_path)

                metrics = compute_basic_metrics(pts_src, pts_dec, args.sample)
                centroid_delta = metrics['centroid_delta']
                bbox_delta = metrics['bbox_delta']

                print(f"[FRAME {frame:04d}] {stem}")
                print(f"  points:  src={metrics['n_src']}  dec={metrics['n_dec']}  diff={metrics['diff']:+d}")
                print(
                    "  centroid diff (m):  "
                    f"x={centroid_delta[0]:+.3f}  y={centroid_delta[1]:+.3f}  z={centroid_delta[2]:+.3f}  ‖Δ‖={metrics['centroid_norm']:.3f}"
                )
                print(
                    "  bbox size Δ (m):   "
                    f"x={bbox_delta[0]:+.3f}  y={bbox_delta[1]:+.3f}  z={bbox_delta[2]:+.3f}"
                )
                print(f"  mean|max dist (m): {metrics['chamfer_mean']}/{metrics['chamfer_max']}")

                node.publish_pair(args.frame_id, pts_src, pts_dec)
                rclpy.spin_once(node, timeout_sec=0.0)
                if interval > 0:
                    time.sleep(interval)
            if not args.loop:
                break
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
