#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
오프라인 파이프라인 (rosbag 재생 → PLY 저장 → Draco 인코딩 → 품질분석)
- bag_to_ply 가 idle-timeout 또는 max-frames 로 자연 종료되면
  추가로 SIGINT 보내지 않고 정상으로 처리하도록 수정
- rosbag 종료 순서/대기 로직 개선
"""

import argparse
import csv
import math
import os
import shlex
import signal
import subprocess as sp
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

# ---------- 유틸 ----------

def _print(*a, **k):
    print(*a, **k)
    sys.stdout.flush()

def _eprint(*a, **k):
    print(*a, file=sys.stderr, **k)
    sys.stderr.flush()

def _popen(cmd, **kwargs):
    _print("[RUN]", " ".join(cmd))
    return sp.Popen(cmd, stdout=sp.PIPE, stderr=sp.STDOUT, text=True, bufsize=1, **kwargs)

def _read_lines(proc, tag):
    """yield line; break if proc ends"""
    assert proc.stdout is not None
    for line in proc.stdout:
        line = line.rstrip("\n")
        print(f"[{tag}] {line}")
        sys.stdout.flush()
        yield line
    # drain ended

def _terminate(proc, name, sig=signal.SIGINT, wait_sec=3.0):
    """send sig then wait; fall back to kill"""
    if proc.poll() is not None:
        return
    _print(f"[CLEANUP] stopping {name} ({signal.Signals(sig).name})…")
    try:
        proc.send_signal(sig)
    except Exception:
        pass
    t0 = time.time()
    while proc.poll() is None and (time.time() - t0) < wait_sec:
        time.sleep(0.1)
    if proc.poll() is None:
        _print(f"[CLEANUP] {name} still alive → SIGKILL")
        try:
            proc.kill()
        except Exception:
            pass


def _safe_float(val):
    if val in (None, "", "nan", "NaN"):
        return None
    try:
        f = float(val)
    except (TypeError, ValueError):
        return None
    if math.isnan(f):
        return None
    return f


def _load_scalar_map(path: Path, value_field: str) -> dict[str, float]:
    mapping: dict[str, float] = {}
    if not path or not path.exists():
        return mapping
    with path.open('r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = (row.get('name') or '').strip()
            if not name:
                continue
            val = _safe_float(row.get(value_field))
            if val is not None:
                mapping[name] = val
    return mapping


def _aggregate_cycle_times(save_log: Path, enc_log: Path,
                           quality_csv: Optional[Path], out_csv: Path) -> None:
    save_map = _load_scalar_map(save_log, 'save_s')
    enc_map = _load_scalar_map(enc_log, 'encode_s')

    rows = []

    if quality_csv and quality_csv.exists():
        with quality_csv.open('r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = (row.get('name') or '').strip()
                if not name:
                    continue
                decode_s = _safe_float(row.get('decode_s'))
                metric_s = _safe_float(row.get('metric_s'))
                save_s = save_map.get(name)
                encode_s = enc_map.get(name)
                parts = [save_s, encode_s, decode_s, metric_s]
                cycle_s = sum(v for v in parts if v is not None) if any(v is not None for v in parts) else None
                rows.append({
                    'name': name,
                    'save_s': save_s,
                    'encode_s': encode_s,
                    'decode_s': decode_s,
                    'metric_s': metric_s,
                    'cycle_s': cycle_s,
                })
    else:
        # 품질 분석을 생략한 경우: save/encode 로그 기반으로만 생성
        all_names = sorted(set(save_map.keys()) | set(enc_map.keys()), key=lambda n: int(n.split('_')[-1]) if n.split('_')[-1].isdigit() else n)
        for name in all_names:
            save_s = save_map.get(name)
            encode_s = enc_map.get(name)
            rows.append({
                'name': name,
                'save_s': save_s,
                'encode_s': encode_s,
                'decode_s': None,
                'metric_s': None,
                'cycle_s': sum(v for v in (save_s, encode_s) if v is not None) if any(v is not None for v in (save_s, encode_s)) else None,
            })

    if not rows:
        return

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open('w', newline='', encoding='utf-8') as f:
        fieldnames = ['name', 'save_s', 'encode_s', 'decode_s', 'metric_s', 'cycle_s']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            cleaned = {}
            for k in fieldnames:
                v = r[k]
                if isinstance(v, float):
                    cleaned[k] = f"{v:.9f}"
                else:
                    cleaned[k] = "" if v is None else v
            writer.writerow(cleaned)

# ---------- 메인 ----------

def main():
    ap = argparse.ArgumentParser(description="Offline pipeline: rosbag→PLY→Draco→Quality")
    # rosbag
    ap.add_argument("--bag", default="", help="rosbag 디렉터리(메타/DB3 포함). 비우면 재생 안 함")
    ap.add_argument("--bag-rate", type=float, default=1.0)
    ap.add_argument("--bag-loop", action="store_true")
    ap.add_argument("--bag-remap", default="", help="ros2 bag play의 remap 규칙 문자열")
    ap.add_argument("--bag-args", nargs="*", default=[], help="ros2 bag play 추가 인자")
    ap.add_argument("--bag-warmup-sec", type=float, default=0.0)

    # saver
    ap.add_argument("--topic", required=True)
    ap.add_argument("--prefix", required=True)
    ap.add_argument("--max-frames", type=int, default=0, dest="frames",
                    help="0=무제한 (bag 끝날 때까지)")
    ap.add_argument("--best-effort", action="store_true")

    # Draco encode/quality
    DEFAULT_CL = 8
    DEFAULT_QP = 12
    DEFAULT_QG = 10

    ap.add_argument("--cl", type=int, default=DEFAULT_CL)
    ap.add_argument("--qp", type=int, default=DEFAULT_QP)
    ap.add_argument("--qg", type=int, default=DEFAULT_QG)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--thresholds", nargs="*", type=float, default=[0.01, 0.03, 0.05])
    ap.add_argument("--decoder", default="draco_decoder")

    # 디렉토리
    ap.add_argument("--ply-dir", default="data/ply_raw")
    ap.add_argument("--drc-dir", default="data/draco_out")
    ap.add_argument("--decoded-dir", default="data/tmp_decoded_ply")
    ap.add_argument("--results-dir", default="data/results")

    ap.add_argument("--ros-domain-id", type=int, default=None,
                    help="하위 프로세스에 전달할 ROS_DOMAIN_ID 덮어쓰기")
    ap.add_argument("--rmw-impl", default=None,
                    help="하위 프로세스에 전달할 RMW_IMPLEMENTATION 덮어쓰기")
    ap.add_argument("--ros-localhost-only", choices=["0", "1"], default=None,
                    help="하위 프로세스에 전달할 ROS_LOCALHOST_ONLY 값")

    ap.add_argument("--keep-decoded", action="store_true", help="QA 단계에서 디코드 PLY를 삭제하지 않음")
    ap.add_argument("--reuse-drc", action="store_true", help="기존 DRC를 재사용(재인코딩 생략)")
    ap.add_argument("--saver-voxel-size", type=float, default=0.0,
                    help="bag_to_ply voxel downsample 크기(m)")
    ap.add_argument("--encoder-extra", action="append", default=[],
                    help="draco_encoder에 넘길 추가 인자 문자열 (예: '--speed 10')")
    ap.add_argument("--fast-preset", action="store_true", help="30FPS 목표용 빠른 설정 적용")
    ap.add_argument("--no-qa", action="store_true", help="품질 분석 단계 생략")

    # ... 기존 인자들 아래에 1줄 추가
    ap.add_argument("--force-tqdm", action="store_true", help="QA 단계(analyze_draco_quality.py)에서 tqdm 강제 표시")


    args = ap.parse_args()

    # VSCode 터미널 등에서 ROS 환경이 누락될 때 대비해 선택적으로 덮어쓰기
    if args.ros_domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    if args.rmw_impl:
        os.environ["RMW_IMPLEMENTATION"] = args.rmw_impl
    if args.ros_localhost_only is not None:
        os.environ["ROS_LOCALHOST_ONLY"] = args.ros_localhost_only

    root = Path.cwd()
    ply_dir = (root / args.ply_dir).resolve()
    drc_dir = (root / args.drc_dir).resolve()
    dec_dir = (root / args.decoded_dir).resolve()
    res_dir = (root / args.results_dir).resolve()
    for d in [ply_dir, drc_dir, dec_dir, res_dir]:
        d.mkdir(parents=True, exist_ok=True)

    extra_tokens = []
    for chunk in args.encoder_extra:
        extra_tokens.extend(shlex.split(str(chunk)))
    args.encoder_extra = extra_tokens

    if args.fast_preset:
        if args.cl == DEFAULT_CL:
            args.cl = 2
        if args.qp == DEFAULT_QP:
            args.qp = 8
        if args.qg == DEFAULT_QG:
            args.qg = 8
        if not args.encoder_extra:
            args.encoder_extra = ["--speed", "11"]

    run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_log_path = res_dir / f"cycle_save_{args.prefix}_{run_ts}.csv"
    enc_log_path = res_dir / f"cycle_encode_{args.prefix}_{run_ts}.csv"

    # 1) rosbag play
    bag_proc = None
    if args.bag:
        bag_abs = (root / args.bag).resolve()
        bag_cmd = ["ros2", "bag", "play", str(bag_abs)]
        if args.bag_rate and args.bag_rate != 1.0:
            bag_cmd += ["--rate", str(args.bag_rate)]
        if args.bag_loop:
            bag_cmd.append("-l")
        if args.bag_remap:
            bag_cmd += ["--remap", args.bag_remap]
        if args.bag_args:
            bag_cmd += args.bag_args

        _print("[ROS2-BAG] starting:", " ".join(bag_cmd))
        bag_proc = _popen(bag_cmd)

        # warmup (선택)
        if args.bag_warmup_sec > 0:
            time.sleep(args.bag_warmup_sec)

    # 2) saver (bag_to_ply.py)
    saver_cmd = [
        sys.executable, str((root / "data" / "bag_to_ply.py").resolve()),
        "--topic", args.topic,
        "--out", str(ply_dir),
        "--prefix", args.prefix,
        "--idle-timeout-sec", "2.0",   # bag 끝나면 2초 후 자동 종료
        "--log-csv", str(save_log_path),
    ]
    if args.saver_voxel_size and args.saver_voxel_size > 0.0:
        saver_cmd += ["--voxel-size", str(args.saver_voxel_size)]
    if args.best_effort:
        saver_cmd.append("--best-effort")
    if args.frames and args.frames > 0:
        saver_cmd += ["--max-frames", str(args.frames)]

    saver_proc = _popen(saver_cmd)

    # saver 진행 감시
    reason = None  # "max", "idle", "proc_end"
    try:
        for line in _read_lines(saver_proc, "PLY"):
            if "Reached max_frames" in line:
                reason = "max"
                break
            if "No messages for idle-timeout" in line:
                reason = "idle"
                break
        # 루프 종료 후에도 프로세스가 끝났는지 확인
        saver_rc = saver_proc.poll()
        if saver_rc is not None and reason is None:
            reason = "proc_end"
    except KeyboardInterrupt:
        reason = "kbd"
    finally:
        # rosbag 먼저 정리
        if bag_proc and bag_proc.poll() is None:
            _print("[PIPELINE] idle-timeout/rosbag_end → stopping ros2 bag first…")
            _terminate(bag_proc, "ros2 bag play", sig=signal.SIGINT, wait_sec=3.0)

        # saver 종료 확인 (이미 끝났으면 신호 보내지 않음)
        saver_rc = saver_proc.poll()
        if saver_rc is None:
            # 아직 살아있으면만 종료 신호
            _terminate(saver_proc, "bag_to_ply", sig=signal.SIGINT, wait_sec=2.0)
            saver_rc = saver_proc.poll()

    # saver 성공 판정:
    # - 정상 종료(rc == 0) → OK
    # - 위에서 우리가 SIGINT 보낸 경우 rc < 0 일 수 있으나,
    #   idle/max/proc_end 등의 reason이 있으면 OK 로 간주
    saver_ok = False
    if saver_rc == 0:
        saver_ok = True
    elif reason in ("idle", "max", "proc_end"):
        # 자연 종료 후 스트림 끊기는 과정에서 returncode가 None→음수로 바뀌는 경우 대비
        saver_ok = True

    if not saver_ok:
        _eprint(f"[FAIL] PLY 저장 단계 실패 (rc={saver_rc})")
        sys.exit(1)

    # 3) Draco 인코딩
    enc_cmd = [
        sys.executable, str((root / "data" / "encode_ply_to_draco.py").resolve()),
        "--in", str(ply_dir),
        "--out", str(drc_dir),
        "--name", args.prefix,
        "--cl", str(args.cl), "--qp", str(args.qp), "--qg", str(args.qg),
        "--log-csv", str(enc_log_path),
    ]
    if not args.reuse_drc:
        enc_cmd.append("--no-skip-existing")
    if args.jobs and args.jobs > 1:
        enc_cmd += ["--workers", str(args.jobs)]
    if args.encoder_extra:
        enc_cmd += ["--extra", *args.encoder_extra]

    _print("[ENC] ", " ".join(enc_cmd))
    enc_rc = sp.call(enc_cmd)
    if enc_rc != 0:
        _eprint(f"[FAIL] Draco 인코딩 단계 실패 (rc={enc_rc})")
        sys.exit(2)

    quality_csv_path = res_dir / f"quality_{args.prefix}_{run_ts}.csv"
    cycle_csv_path = res_dir / f"cycle_{args.prefix}_{run_ts}.csv"
    if not args.no_qa:
        qa_cmd = [
            sys.executable, str((root / "analysis" / "analyze_draco_quality.py").resolve()),
            "--ply_dir", str(ply_dir),
            "--drc_dir", str(drc_dir),
            "--decoded_dir", str(dec_dir),
            "--results_dir", str(res_dir),
            "--prefix", args.prefix,
            "--decoder", args.decoder,
            "--thresholds", *[str(t) for t in args.thresholds],
            "--run-ts", run_ts
        ]
        if getattr(args, "force_tqdm", False):
            qa_cmd.append("--force-tqdm")
        if getattr(args, "keep_decoded", False):
            qa_cmd.append("--keep-decoded")
        if getattr(args, "jobs", None):
            qa_cmd += ["--decode-workers", str(args.jobs),
                       "--metric-workers", str(args.jobs)]
        _print("[QA]  ", " ".join(qa_cmd))
        qa_rc = sp.call(qa_cmd)
        if qa_rc != 0:
            _eprint(f"[WARN] 품질 분석 스크립트가 비정상 종료(rc={qa_rc}). 결과 파일을 확인하세요.")
            sys.exit(3)

        try:
            _aggregate_cycle_times(save_log_path, enc_log_path, quality_csv_path, cycle_csv_path)
            if cycle_csv_path.exists():
                _print(f"[CYCLE] {cycle_csv_path}")
        except Exception as ex:
            _eprint(f"[WARN] 사이클 시간 집계 실패: {ex}")
    else:
        try:
            _aggregate_cycle_times(save_log_path, enc_log_path, None, cycle_csv_path)
            if cycle_csv_path.exists():
                _print(f"[CYCLE] {cycle_csv_path}")
        except Exception as ex:
            _eprint(f"[WARN] 사이클 시간 집계 실패: {ex}")

    _print("[OK] 파이프라인 완료")

if __name__ == "__main__":
    main()
