#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PLY(original) vs DRC(Draco) 품질/성능 비교 (멀티코어 + 진행바 강화판)

기능
- drc 디렉토리의 .drc 를 draco_decoder 로 병렬 디코딩(.ply 임시 생성)
- 원본 ply 와 1:1 매칭하여 기하 오차(양방향 최근접거리 기반, Chamfer-like),
  파일 크기/압축배율, 디코드 시간/디코드 FPS 계산
- (옵션) voxel 다운샘플, 통계적 아웃라이어 제거, 무작위 샘플링
- 임계치(예: 0.01/0.03/0.05m) 통과율 계산
- 결과 CSV + 요약 MD + 누락/불일치 진단

사용 예)
  cd ~/draco-ros2-roundtrip
  python3 analysis/analyze_draco_quality.py \
    --ply_dir data/ply_raw \
    --drc_dir data/draco_out \
    --decoded_dir data/tmp_decoded_ply \
    --results_dir data/results \
    --prefix sample2 \
    --thresholds 0.01 0.03 0.05 \
    --decode-workers 8 \
    --metric-workers 8 \
    --force-tqdm

참고
- Open3D 설치 권장: pip install open3d
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
import csv
import math
import random
import subprocess as sp
from pathlib import Path
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import List, Tuple, Dict, Optional

import numpy as np

# 선택 의존성
_HAVE_TQDM = False
_HAVE_O3D  = False
try:
    from tqdm import tqdm  # type: ignore
    _HAVE_TQDM = True
except Exception:
    pass

try:
    import open3d as o3d  # type: ignore
    _HAVE_O3D = True
except Exception:
    pass


# ---------- 유틸 ----------
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)
    sys.stderr.flush()


def which(cmd: str) -> Optional[str]:
    for p in os.environ.get("PATH", "").split(os.pathsep):
        c = Path(p) / cmd
        if c.exists() and os.access(c, os.X_OK):
            return str(c)
    return None


def ensure_dir(d: Path):
    d.mkdir(parents=True, exist_ok=True)


def find_pairs(ply_dir: Path, drc_dir: Path, prefix: str) -> List[Tuple[Path, Path, str]]:
    """
    prefix_0000000000.ply ↔ prefix_0000000000.drc 형태 매칭
    반환: [(ply_path, drc_path, stem), ...]  where stem = 'prefix_##########'
    """
    pat = re.compile(rf"^{re.escape(prefix)}_(\d+)\.(ply|drc)$")
    ply_map: Dict[str, Path] = {}
    drc_map: Dict[str, Path] = {}

    for p in sorted(ply_dir.glob("*.ply")):
        m = pat.match(p.name)
        if m:
            ply_map[m.group(1)] = p

    for d in sorted(drc_dir.glob("*.drc")):
        m = pat.match(d.name)
        if m:
            drc_map[m.group(1)] = d

    idxs = sorted(set(ply_map.keys()) & set(drc_map.keys()), key=lambda x: int(x))
    return [(ply_map[i], drc_map[i], f"{prefix}_{i}") for i in idxs]


def parse_stat_outlier(s: Optional[str]) -> Optional[Tuple[int, float]]:
    """
    'k:20,nb_std:2.0' 또는 '20,2.0' 형태 지원
    None 또는 빈 문자열이면 사용 안 함
    """
    if not s:
        return None
    if "," in s:
        parts = s.split(",")
    elif ":" in s:
        parts = s.replace(" ", "").split(",")
    else:
        parts = s.split(",")

    k = None
    nb = None
    for p in parts:
        p = p.strip()
        if not p:
            continue
        if ":" in p:
            key, val = p.split(":")
            key = key.strip().lower()
            val = val.strip()
            if key in ("k", "nn", "neighbors"):
                k = int(val)
            elif key in ("nb_std", "std", "sigma"):
                nb = float(val)
        else:
            # 위치기반: 첫번째 = k, 두번째 = nb_std
            if k is None:
                k = int(p)
            elif nb is None:
                nb = float(p)
    if k is None or nb is None:
        return None
    return (int(k), float(nb))


# ---------- 디코딩 ----------
def _decode_one(decoder: str, drc: Path, out_ply: Path) -> Tuple[str, float, int]:
    """
    하나의 DRC를 PLY로 디코딩.
    반환: (stem, decode_seconds, rc)
    """
    t0 = time.perf_counter()
    cmd = [decoder, "-i", str(drc), "-o", str(out_ply)]
    proc = sp.run(cmd, stdout=sp.PIPE, stderr=sp.PIPE, text=True)
    dt = time.perf_counter() - t0
    rc = proc.returncode
    return (drc.stem, dt, rc)


# ---------- 메트릭 ----------
def _load_pcd_points_o3d(path: Path) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(pcd.points, dtype=np.float32)


def _preprocess_xyz(
    xyz: np.ndarray,
    voxel_size: Optional[float],
    stat_outlier: Optional[Tuple[int, float]],
    max_samples: Optional[int],
    seed: Optional[int],
) -> np.ndarray:
    if xyz.size == 0:
        return xyz

    pts = xyz
    if _HAVE_O3D:
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
        # Voxel downsample
        if voxel_size and voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)
        # Statistical outlier removal
        if stat_outlier:
            k, nb = stat_outlier
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=int(k), std_ratio=float(nb))
        pts = np.asarray(pcd.points, dtype=np.float32)

    # Random subsample (uniform)
    if max_samples and max_samples > 0 and len(pts) > max_samples:
        rng = np.random.default_rng(seed if seed is not None else None)
        idx = rng.choice(len(pts), size=max_samples, replace=False)
        pts = pts[idx]

    return pts


def _nn_stats(a: np.ndarray, kd_b: o3d.geometry.KDTreeFlann) -> Tuple[float, float]:
    if len(a) == 0:
        return float("nan"), float("nan")
    dists = np.empty((len(a),), dtype=np.float64)
    for i, p in enumerate(a):
        _, idx, dist2 = kd_b.search_knn_vector_3d(p.astype(np.float64), 1)
        dists[i] = math.sqrt(dist2[0]) if idx else math.nan
    if not len(dists):
        return float("nan"), float("nan")
    finite = dists[np.isfinite(dists)]
    if finite.size == 0:
        return float("nan"), float("nan")
    return float(np.mean(finite)), float(np.max(finite))


def _metric_one(
    stem: str,
    ply_src: Path,
    ply_dec: Path,
    thresholds: List[float],
    voxel_size: Optional[float],
    stat_outlier: Optional[Tuple[int, float]],
    max_samples: Optional[int],
    seed: Optional[int],
) -> Dict[str, object]:
    t_metric = time.perf_counter()
    try:
        A = _load_pcd_points_o3d(ply_src)
        B = _load_pcd_points_o3d(ply_dec)

        A = _preprocess_xyz(A, voxel_size, stat_outlier, max_samples, seed)
        B = _preprocess_xyz(B, voxel_size, stat_outlier, max_samples, seed)

        n_src, n_dec = len(A), len(B)
        if n_src == 0 or n_dec == 0:
            raise ValueError("empty cloud after preprocess")

        # KDTree
        a_p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(A.astype(np.float64)))
        b_p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(B.astype(np.float64)))
        kdA = o3d.geometry.KDTreeFlann(a_p)
        kdB = o3d.geometry.KDTreeFlann(b_p)

        mean_ab, max_ab = _nn_stats(A, kdB)
        mean_ba, max_ba = _nn_stats(B, kdA)
        haus = float("nan")
        finite_max = [v for v in (max_ab, max_ba) if math.isfinite(v)]
        if finite_max:
            haus = float(max(finite_max))

        # Chamfer-like 분포(필요 최소치만): 단방향 분포를 샘플해서 RMS/median/hausdorff 근사
        # 정확도를 더 높이고 싶으면 양방향 전체 분포를 수집하도록 확장 가능
        # 여기서는 대략적인 지표 용도이므로 mean_ab/mean_ba만 평균값 중심으로 사용
        # 추가로, pass rate는 보수적으로 양방향 모두 통과 조건으로 판정
        pr = {}
        for thr in thresholds:
            ok = (mean_ab <= thr) and (mean_ba <= thr)
            pr[f"pass_{thr}m"] = 1.0 if ok else 0.0

        out = {
            "name": stem,
            "src": ply_src.name,
            "drc_decoded": ply_dec.name,
            "n_src": n_src,
            "n_dec": n_dec,
            "mean_src_to_dec": mean_ab,
            "mean_dec_to_src": mean_ba,
            # 간단 요약 지표(양방향 평균을 'chamfer_mean' 근사치로 둔다)
            "chamfer_mean": float((mean_ab + mean_ba) / 2.0),
            # 아래 값들은 정밀 계산이 아니라 표기용 자리. 필요시 전방위 분포 수집으로 교체 가능.
            "chamfer_med": float("nan"),
            "chamfer_rms": float("nan"),
            "hausdorff": haus,
        }
        out.update(pr)
        out["status"] = "ok"
        out["metric_s"] = float(time.perf_counter() - t_metric)
        return out

    except Exception as ex:
        return {
            "name": stem,
            "src": ply_src.name,
            "drc_decoded": ply_dec.name,
            "n_src": "",
            "n_dec": "",
            "mean_src_to_dec": "",
            "mean_dec_to_src": "",
            "chamfer_mean": "",
            "chamfer_med": "",
            "chamfer_rms": "",
            "hausdorff": "",
            **{f"pass_{t}m": "" for t in thresholds},
            "status": f"metric_fail: {type(ex).__name__}: {ex}",
            "metric_s": "",
        }


# ---------- 메인 ----------
def main():
    ap = argparse.ArgumentParser(description="Draco 품질/성능 분석 (병렬 + 진행바)")
    ap.add_argument("--ply_dir", default="data/ply_raw", help="원본 PLY 디렉토리")
    ap.add_argument("--drc_dir", default="data/draco_out", help="DRC 디렉토리")
    ap.add_argument("--decoded_dir", default="data/tmp_decoded_ply", help="복원 PLY 저장 디렉토리")
    ap.add_argument("--results_dir", default="data/results", help="결과 저장 디렉토리")
    ap.add_argument("--prefix", required=True, help="파일 접두어 (예: sample2)")
    ap.add_argument("--decoder", default=None, help="draco_decoder 경로(미지정 시 PATH/관례 탐색)")
    ap.add_argument("--thresholds", nargs="*", type=float, default=[0.01, 0.03, 0.05],
                    help="오차 임계치(m) 목록")
    ap.add_argument("--limit", type=int, default=0, help="0이면 전부, 아니면 앞에서 N개만")

    # 성능 옵션
    ap.add_argument("--decode-workers", type=int, default=1, help="DRC→PLY 디코드 병렬 프로세스 수")
    ap.add_argument("--metric-workers", type=int, default=1, help="품질 계산 병렬 프로세스 수")

    # 전처리 옵션
    ap.add_argument("--voxel-size", type=float, default=0.0, help="voxel 다운샘플 크기(m), 0=미사용")
    ap.add_argument("--stat-outlier", default=None,
                    help="통계적 아웃라이어 제거 설정 (예: 'k:20,nb_std:2.0' 또는 '20,2.0')")
    ap.add_argument("--max-samples", type=int, default=0, help="무작위 샘플링 최대 포인트 수(0=무제한)")
    ap.add_argument("--seed", type=int, default=None, help="샘플링 시드")
    ap.add_argument("--run-ts", default=None, help="결과 파일명에 사용할 타임스탬프(YYYYmmdd_HHMMSS)")

    # 기타
    ap.add_argument("--keep-decoded", action="store_true", help="디코드 PLY 유지")
    ap.add_argument("--no-tqdm", action="store_true", help="진행바 끄기")
    ap.add_argument("--force-tqdm", action="store_true", help="설치돼 있으면 진행바 강제 표시")

    args = ap.parse_args()

    ply_dir = Path(args.ply_dir).resolve()
    drc_dir = Path(args.drc_dir).resolve()
    dec_dir = Path(args.decoded_dir).resolve()
    res_dir = Path(args.results_dir).resolve()
    ensure_dir(dec_dir)
    ensure_dir(res_dir)

    # tqdm 사용 여부
    use_tqdm = _HAVE_TQDM and (args.force_tqdm or (not args.no_tqdm))

    # draco_decoder 찾기
    decoder_path = None
    # 1) 인자인 파일/폴더
    if args.decoder:
        p = Path(args.decoder).expanduser().resolve()
        if p.is_file() and os.access(str(p), os.X_OK):
            decoder_path = str(p)
        elif p.is_dir():
            cand = p / "draco_decoder"
            if cand.exists() and os.access(str(cand), os.X_OK):
                decoder_path = str(cand)
    # 2) 환경변수
    if decoder_path is None:
        env = os.environ.get("DRACO_DECODER")
        if env:
            p = Path(env).expanduser().resolve()
            if p.is_file() and os.access(str(p), os.X_OK):
                decoder_path = str(p)
    # 3) PATH
    if decoder_path is None:
        w = which("draco_decoder")
        if w:
            decoder_path = w
    # 4) 관례 위치
    if decoder_path is None:
        home = Path.home()
        for c in [
            home / "draco" / "build" / "bin" / "draco_decoder",
            home / "draco" / "build" / "draco_decoder",
            Path(__file__).resolve().parents[1] / "draco" / "bin" / "draco_decoder",
        ]:
            if c.exists() and os.access(str(c), os.X_OK):
                decoder_path = str(c)
                break
    if decoder_path is None:
        eprint("[ERROR] draco_decoder 를 찾을 수 없습니다. --decoder 또는 PATH/DRACO_DECODER 확인.")
        sys.exit(2)

    # 페어 매칭
    pairs = find_pairs(ply_dir, drc_dir, args.prefix)
    if args.limit and args.limit > 0:
        pairs = pairs[: args.limit]
    if not pairs:
        eprint("[WARN] 매칭되는 (PLY, DRC) 쌍이 없습니다.")
        sys.exit(0)

    # 누락/불일치 진단용 집합
    ply_stems_all = {p.stem for p in ply_dir.glob(f"{args.prefix}_*.ply")}
    drc_stems_all = {d.stem for d in drc_dir.glob(f"{args.prefix}_*.drc")}
    matched_stems = {stem for _, _, stem in pairs}
    only_in_ply = sorted(ply_stems_all - matched_stems, key=lambda s: int(s.split("_")[-1]) if s.split("_")[-1].isdigit() else s)
    only_in_drc = sorted(drc_stems_all - matched_stems, key=lambda s: int(s.split("_")[-1]) if s.split("_")[-1].isdigit() else s)

    # 1) 디코딩 (병렬)
    dec_results: Dict[str, Dict[str, object]] = {}
    tasks = []
    with ProcessPoolExecutor(max_workers=max(1, args.decode_workers)) as pool:
        if use_tqdm:
            pbar = tqdm(total=len(pairs), unit="file", desc="Decoding", leave=False)
        else:
            pbar = None

        futures = []
        for ply_path, drc_path, stem in pairs:
            out_ply = dec_dir / f"{stem}.decoded.ply"
            futures.append(pool.submit(_decode_one, decoder_path, drc_path, out_ply))

        for fut in as_completed(futures):
            stem, dt, rc = fut.result()
            dec_results[stem] = {"decode_s": float(dt), "decode_fps": (1.0 / dt) if dt > 0 else float("nan"), "rc": int(rc)}
            if pbar:
                pbar.update(1)
        if pbar:
            pbar.close()

    # 2) 품질 계산 (병렬)
    stat_out = parse_stat_outlier(args.stat_outlier)
    metric_rows: List[Dict[str, object]] = []

    def submit_metric(pool, *, stem, ply_src, ply_dec):
        return pool.submit(
            _metric_one,
            stem,
            ply_src,
            ply_dec,
            args.thresholds,
            args.voxel_size if args.voxel_size > 0 else None,
            stat_out,
            args.max_samples if args.max_samples > 0 else None,
            args.seed,
        )

    with ProcessPoolExecutor(max_workers=max(1, args.metric_workers)) as pool:
        if use_tqdm:
            pbar = tqdm(total=len(pairs), unit="pair", desc="Quality", leave=False)
        else:
            pbar = None

        futures = []
        for ply_path, _, stem in pairs:
            dec_p = dec_dir / f"{stem}.decoded.ply"
            futures.append(submit_metric(pool, stem=stem, ply_src=ply_path, ply_dec=dec_p))

        for fut in as_completed(futures):
            metric_rows.append(fut.result())
            if pbar:
                pbar.update(1)
        if pbar:
            pbar.close()

    # 3) 결과 합치기 + 크기/압축배율
    rows = []
    for ply_path, drc_path, stem in pairs:
        base = {
            "name": stem,
            "ply": ply_path.name,
            "drc": drc_path.name,
            "size_ply": ply_path.stat().st_size if ply_path.exists() else "",
            "size_drc": drc_path.stat().st_size if drc_path.exists() else "",
        }
        dec = dec_results.get(stem, {})
        base["decode_s"] = dec.get("decode_s", "")
        base["decode_fps"] = dec.get("decode_fps", "")
        # metric row 찾기
        m = next((r for r in metric_rows if r.get("name") == stem), None)
        if m:
            base.update(m)
        # 압축배율
        try:
            sz_p = float(base["size_ply"])
            sz_d = float(base["size_drc"])
            base["ratio_ply_over_drc"] = (sz_p / sz_d) if (sz_d > 0) else float("nan")
        except Exception:
            base["ratio_ply_over_drc"] = ""
        rows.append(base)

    # 4) CSV 저장
    ts = args.run_ts if args.run_ts else datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = res_dir / f"quality_{args.prefix}_{ts}.csv"
    fieldnames = sorted({k for r in rows for k in r.keys()})
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in sorted(rows, key=lambda x: int(x["name"].split("_")[-1]) if x.get("name","").split("_")[-1].isdigit() else x.get("name","")):
            w.writerow(r)

    # 5) 요약 MD
    md_path = res_dir / f"summary_{args.prefix}_{ts}.md"

    def _flt_mean(col: str):
        vals = [r.get(col) for r in rows if isinstance(r.get(col), (int, float))]
        return float(np.mean(vals)) if vals else float("nan")

    def _flt_max(col: str):
        vals = [r.get(col) for r in rows if isinstance(r.get(col), (int, float))]
        return float(np.max(vals)) if vals else float("nan")

    def fmt(x): 
        return f"{x:.6g}" if isinstance(x, (int, float)) and math.isfinite(x) else "nan"

    n_all = len(rows)
    mean_ratio = _flt_mean("ratio_ply_over_drc")
    mean_decode_fps = _flt_mean("decode_fps")
    mean_chamfer = _flt_mean("chamfer_mean")
    haus_max = _flt_max("hausdorff")

    # 임계치 통과율 평균
    thr_lines = []
    for thr in args.thresholds:
        col = f"pass_{thr}m"
        vals = [r.get(col) for r in rows if isinstance(r.get(col), (int, float))]
        thr_lines.append(f"- 오차 ≤ {thr} m 비율(평균): {fmt(np.mean(vals)*100.0)} %")

    # 누락/불일치 진단
    diag = []
    if only_in_ply:
        diag.append(f"- PLY 전용({len(only_in_ply)}): " + ", ".join(only_in_ply[:10]) + (" ..." if len(only_in_ply) > 10 else ""))
    if only_in_drc:
        diag.append(f"- DRC 전용({len(only_in_drc)}): " + ", ".join(only_in_drc[:10]) + (" ..." if len(only_in_drc) > 10 else ""))

    with open(md_path, "w", encoding="utf-8") as f:
        f.write(f"# Draco 품질/성능 요약 ({args.prefix})\n\n")
        f.write(f"- 샘플 수(매칭): {n_all}\n")
        f.write(f"- 평균 압축배율(PLY/DRC): {fmt(mean_ratio)}x\n")
        f.write(f"- 평균 디코드 FPS: {fmt(mean_decode_fps)}\n")
        f.write(f"- Chamfer 근사(mean): {fmt(mean_chamfer)} m\n")
        f.write(f"- Hausdorff 근사(max): {fmt(haus_max)} m\n")
        for line in thr_lines:
            f.write(line + "\n")
        f.write("\n## 누락/불일치 진단\n")
        if diag:
            for d in diag:
                f.write(d + "\n")
        else:
            f.write("- 불일치 없음\n")
        f.write(f"\n- CSV: `{csv_path}`\n")

    # 6) 임시 디코드 정리
    if not args.keep_decoded:
        try:
            for _, _, stem in pairs:
                p = dec_dir / f"{stem}.decoded.ply"
                if p.exists():
                    p.unlink()
        except Exception:
            pass

    print(f"[OK] CSV  : {csv_path}")
    print(f"[OK] SUMM : {md_path}")
    print("[TIP] --decode-workers / --metric-workers 를 올리면 CPU 활용이 좋아집니다. "
          "Open3D 가속이 필수이니 미설치 시 `pip install open3d` 하세요.")


if __name__ == "__main__":
    # 출력 버퍼링 완화(프로그레스/로그 실시간)
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass
    main()
