#!/usr/bin/env python3
"""Realtime PointCloud2 diff monitor for source/decoded PLY playback."""

from __future__ import annotations

import argparse
import math
import threading
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import numpy as np
from numpy.linalg import norm
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

try:
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:  # pragma: no cover - optional dep
    _HAVE_O3D = False


@dataclass
class CloudInfo:
    stamp_sec: float
    frame_id: str
    points: np.ndarray


def _to_xyz(msg: PointCloud2) -> np.ndarray:
    try:
        arr = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape((-1, 3))
        elif arr.shape[1] > 3:
            arr = arr[:, :3]
        return arr
    except Exception:
        pts = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            try:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
            except Exception:
                x = float(p['x'])
                y = float(p['y'])
                z = float(p['z'])
            pts.append((x, y, z))
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)


class DiffMonitor(Node):
    def __init__(self, src_topic: str, dec_topic: str, max_delay: float,
                 sample: int, use_icp: bool):
        super().__init__('pcd_diff_monitor')

        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._max_delay = Duration(seconds=max_delay)
        self._sample = sample
        self._use_icp = use_icp and _HAVE_O3D

        self._buffer_src: Dict[int, CloudInfo] = {}
        self._buffer_dec: Dict[int, CloudInfo] = {}
        self._lock = threading.Lock()
        self._frame_index = 0

        self.create_subscription(PointCloud2, src_topic, self._on_src, qos)
        self.create_subscription(PointCloud2, dec_topic, self._on_dec, qos)
        self.get_logger().info(
            f"Listening src='{src_topic}' decoded='{dec_topic}' max_delay={max_delay}s "
            f"sample={sample} ICP={'on' if self._use_icp else 'off'}")

    def _on_src(self, msg: PointCloud2) -> None:
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        key = (sec << 32) | nsec
        stamp_sec = sec + nsec * 1e-9
        with self._lock:
            self._buffer_src[key] = CloudInfo(
                stamp_sec=stamp_sec,
                frame_id=msg.header.frame_id,
                points=_to_xyz(msg))
        self._try_match()

    def _on_dec(self, msg: PointCloud2) -> None:
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        key = (sec << 32) | nsec
        stamp_sec = sec + nsec * 1e-9
        with self._lock:
            self._buffer_dec[key] = CloudInfo(
                stamp_sec=stamp_sec,
                frame_id=msg.header.frame_id,
                points=_to_xyz(msg))
        self._try_match()

    def _try_match(self) -> None:
        with self._lock:
            if not self._buffer_src or not self._buffer_dec:
                return
            # naive: find closest timestamp pair
            for key_src, info_src in list(self._buffer_src.items()):
                best_key = None
                best_dt = Duration(seconds=999)
                for key_dec, info_dec in self._buffer_dec.items():
                    dt_seconds = abs(info_src.stamp_sec - info_dec.stamp_sec)
                    dt_duration = Duration(seconds=dt_seconds)
                    if dt_duration < best_dt:
                        best_dt = dt_duration
                        best_key = key_dec
                if best_key is not None and best_dt <= self._max_delay:
                    info_dec = self._buffer_dec.pop(best_key)
                    self._buffer_src.pop(key_src, None)
                    self._report(info_src, info_dec, best_dt.nanoseconds * 1e-9)
                    break
            # drop stale entries
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            stale = [k for k, v in self._buffer_src.items() if (now_sec - v.stamp_sec) > self._max_delay.nanoseconds * 1e-9]
            for k in stale:
                self._buffer_src.pop(k, None)
            stale = [k for k, v in self._buffer_dec.items() if (now_sec - v.stamp_sec) > self._max_delay.nanoseconds * 1e-9]
            for k in stale:
                self._buffer_dec.pop(k, None)

    def _report(self, src: CloudInfo, dec: CloudInfo, dt: float) -> None:
        n_src = len(src.points)
        n_dec = len(dec.points)
        if n_src == 0 or n_dec == 0:
            self.get_logger().warning(
                f"Empty cloud encountered (src={n_src}, dec={n_dec}); skipping")
            return

        pts_src = src.points
        pts_dec = dec.points

        if self._sample > 0:
            rng = np.random.default_rng()
            if len(pts_src) > self._sample:
                pts_src = pts_src[rng.choice(len(pts_src), self._sample, replace=False)]
            if len(pts_dec) > self._sample:
                pts_dec = pts_dec[rng.choice(len(pts_dec), self._sample, replace=False)]

        centroid_src = pts_src.mean(axis=0)
        centroid_dec = pts_dec.mean(axis=0)
        centroid_diff = centroid_dec - centroid_src

        bbox_src = np.vstack((pts_src.min(axis=0), pts_src.max(axis=0)))
        bbox_dec = np.vstack((pts_dec.min(axis=0), pts_dec.max(axis=0)))

        aligned = False
        mean_dist = float('nan')
        max_dist = float('nan')

        if len(pts_src) == len(pts_dec):
            diffs = np.linalg.norm(pts_src - pts_dec, axis=1)
            mean_dist = float(np.mean(diffs))
            max_dist = float(np.max(diffs))
            aligned = True
        elif self._use_icp:
            pcd_src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_src.astype(np.float64)))
            pcd_dec = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_dec.astype(np.float64)))
            reg = o3d.pipelines.registration.registration_icp(
                pcd_dec, pcd_src, 0.5,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
            )
            transformed = np.asarray(
                np.asarray(pcd_dec.transform(reg.transformation).points),
                dtype=np.float32,
            )
            m = min(len(pts_src), len(transformed))
            diffs = np.linalg.norm(pts_src[:m] - transformed[:m], axis=1)
            mean_dist = float(np.mean(diffs))
            max_dist = float(np.max(diffs))
            aligned = True
        else:
            m = min(len(pts_src), len(pts_dec))
            if m > 0:
                diffs = np.linalg.norm(pts_src[:m] - pts_dec[:m], axis=1)
                mean_dist = float(np.mean(diffs))
                max_dist = float(np.max(diffs))
                aligned = True

        centroid_norm = norm(centroid_diff)
        bbox_src_size = bbox_src[1] - bbox_src[0]
        bbox_dec_size = bbox_dec[1] - bbox_dec[0]
        bbox_size_delta = bbox_dec_size - bbox_src_size

        mean_str = "n/a"
        max_str = "n/a"
        if aligned and not math.isnan(mean_dist):
            mean_str = f"{mean_dist:.3f}"
            max_str = f"{max_dist:.3f}"

        self._frame_index += 1
        self.get_logger().info(
            f"[Frame {self._frame_index:04d}] delay={dt*1e3:.1f} ms\n"
            f"  points:  src={n_src}  dec={n_dec}  diff={n_dec - n_src:+d}\n"
            f"  centroid diff (m):  x={centroid_diff[0]:+.3f}  y={centroid_diff[1]:+.3f}  z={centroid_diff[2]:+.3f}  ‖Δ‖={centroid_norm:.3f}\n"
            f"  bbox size Δ (m):   x={bbox_size_delta[0]:+.3f}  y={bbox_size_delta[1]:+.3f}  z={bbox_size_delta[2]:+.3f}\n"
            f"  mean|max dist (m): {mean_str}/{max_str}"
        )


def main(argv: Optional[Iterable[str]] = None) -> None:
    ap = argparse.ArgumentParser(description="Realtime PointCloud diff monitor")
    ap.add_argument("--src", required=True, help="원본 PointCloud2 토픽")
    ap.add_argument("--decoded", required=True, help="디코드 PointCloud2 토픽")
    ap.add_argument("--max-delay", type=float, default=0.1, help="매칭 허용 지연 (s)")
    ap.add_argument("--sample", type=int, default=100000,
                    help="계산에 사용할 최대 포인트 수(0=모두)")
    ap.add_argument("--icp", action="store_true", help="포인트 수 다를 때 ICP 보정")
    args = ap.parse_args(argv)

    rclpy.init()
    try:
        node = DiffMonitor(args.src, args.decoded, args.max_delay,
                           args.sample, args.icp)
        rclpy.spin(node)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
