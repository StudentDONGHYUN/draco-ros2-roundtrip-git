#!/usr/bin/env python3
"""Replay original & decoded PLY pairs as PointCloud2 topics for RViz comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from draco_roundtrip.utils import collect_matching_pairs, load_points


class PairReplay(Node):
    def __init__(self, pairs: list[tuple[Path, Path, str]], frame_id: str,
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

        xyz_src = load_points(orig)
        xyz_dec = load_points(dec)
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


def build_arg_parser() -> argparse.ArgumentParser:
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
    return ap


def main(argv: Iterable[str] | None = None) -> None:
    args = build_arg_parser().parse_args(argv)

    orig_dir = Path(args.orig_dir).expanduser().resolve()
    dec_dir = Path(args.decoded_dir).expanduser().resolve()

    pairs = collect_matching_pairs(orig_dir, dec_dir, args.prefix,
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
