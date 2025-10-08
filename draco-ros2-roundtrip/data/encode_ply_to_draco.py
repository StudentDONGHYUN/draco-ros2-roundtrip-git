#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
encode_ply_to_draco.py  (fixed)
- data/ply_raw/<name>_*.ply 만 골라 동일 stem 으로 data/draco_out/<name>_*.drc 생성
- 출력 파일명은 원본 PLY의 stem을 보존: <name>_<10자리>.ply -> <name>_<10자리>.drc
- --name 미지정 시 모든 *.ply 대상이지만, 가능한 한 --name 사용 권장
"""

from __future__ import annotations
import argparse, csv, os, sys, subprocess, time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from shutil import which
from datetime import datetime

PAD = 10  # (참고용) 0패딩 자릿수

def is_exe(p: Path) -> bool:
    try:
        return p.is_file() and os.access(str(p), os.X_OK)
    except Exception:
        return False

def find_in_dir(d: Path) -> Path | None:
    cand = d / "draco_encoder"
    return cand if is_exe(cand) else None

def find_draco_encoder(user_hint: str | None) -> Path | None:
    if user_hint:
        p = Path(user_hint).expanduser().resolve()
        if p.is_file() and is_exe(p):
            return p
        if p.is_dir():
            f = find_in_dir(p);  return f
    env_path = os.environ.get("DRACO_ENCODER")
    if env_path:
        p = Path(env_path).expanduser().resolve()
        if is_exe(p):
            return p
    w = which("draco_encoder")
    if w:
        p = Path(w).resolve()
        if is_exe(p):
            return p
    home = Path.home()
    project_root = Path(__file__).resolve().parents[1]
    for c in [
        home / "draco" / "build" / "bin" / "draco_encoder",
        home / "draco" / "build" / "draco_encoder",
        project_root / "draco" / "bin" / "draco_encoder",
    ]:
        if is_exe(c):
            return c
    return None

def encode_one(encoder: Path, src_ply: Path, out_dir: Path,
               cl: int, qp: int, qg: int, extra: list[str], skip_existing: bool) -> tuple[Path, int, str, float]:
    out_path = out_dir / (src_ply.stem + ".drc")  # 원본 stem 보존
    if skip_existing and out_path.exists():
        return out_path, 0, "skip-existing", 0.0
    cmd = [str(encoder), "-i", str(src_ply), "-o", str(out_path),
           "-cl", str(cl), "-qp", str(qp), "-qg", str(qg)]
    if extra:
        cmd.extend(extra)
    t0 = time.perf_counter()
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    dt = time.perf_counter() - t0
    msg = (proc.stderr.strip() or proc.stdout.strip() or "")
    return out_path, proc.returncode, msg, dt

def main():
    ap = argparse.ArgumentParser(description="Batch encode PLY -> Draco (.drc)")
    ap.add_argument("--in",  dest="indir",  default="./ply_raw",   help="입력 PLY 디렉터리 (기본: ./ply_raw)")
    ap.add_argument("--out", dest="outdir", default="./draco_out", help="출력 DRC 디렉터리 (기본: ./draco_out)")
    ap.add_argument("--name", required=False, help="대상 접두어(prefix). 지정 시 '<name>_*.ply'만 인코딩")
    ap.add_argument("--draco", dest="draco_hint", default=None, help="draco_encoder 경로(파일/디렉토리)")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 4, help="병렬 작업 수")
    ap.add_argument("--cl", type=int, default=8,  help="-cl compress level")
    ap.add_argument("--qp", type=int, default=12, help="-qp position quantization bits")
    ap.add_argument("--qg", type=int, default=10, help="-qg generic quantization bits")
    ap.add_argument("--skip-existing", action="store_true", default=True, help="이미 존재하는 .drc는 건너뜀")
    ap.add_argument("--no-skip-existing", dest="skip_existing", action="store_false", help="기존 파일도 다시 인코딩")
    ap.add_argument("--extra", nargs=argparse.REMAINDER, default=[], help="draco_encoder에 그대로 넘길 추가 인자")
    ap.add_argument("--log-csv", default=None, help="프레임별 인코드 시간 로그 CSV")
    args = ap.parse_args()

    indir  = Path(args.indir).expanduser().resolve()
    outdir = Path(args.outdir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    log_path = Path(args.log_csv).expanduser().resolve() if args.log_csv else None
    encode_timings: list[tuple[str, float]] = []
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)

    encoder = find_draco_encoder(args.draco_hint)
    if not encoder:
        print("[ERROR] draco_encoder 를 찾을 수 없습니다.", file=sys.stderr)
        print("  예) --draco ~/draco/build/bin  또는 DRACO_ENCODER=/path/to/draco_encoder", file=sys.stderr)
        sys.exit(1)
    os.environ["PATH"] = f"{encoder.parent}:{os.environ.get('PATH','')}"

    # 입력 파일 목록 수집(+필터)
    if args.name:
        ply_files = sorted(indir.glob(f"{args.name}_*.ply"))
    else:
        ply_files = sorted(indir.glob("*.ply"))

    if not ply_files:
        hint = f"{indir} (prefix='{args.name}')" if args.name else f"{indir}"
        print(f"[WARN] 대상 PLY가 없습니다: {hint}")
        sys.exit(0)

    print(f"[INFO] encoder: {encoder}")
    print(f"[INFO] in : {indir} (총 {len(ply_files)}개, prefix={args.name or 'ALL'})")
    print(f"[INFO] out: {outdir}")
    print(f"[INFO] opts: -cl {args.cl} -qp {args.qp} -qg {args.qg}  workers={args.workers}  skip_existing={args.skip_existing}")
    if args.extra:
        print(f"[INFO] extra args -> {' '.join(args.extra)}")

    futures = []
    ok, fail, skipped = 0, 0, 0
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as ex:
        for ply in ply_files:
            futures.append(ex.submit(
                encode_one, encoder, ply, outdir, args.cl, args.qp, args.qg, args.extra, args.skip_existing
            ))
        for fut in as_completed(futures):
            out_path, rc, msg, dt = fut.result()
            if rc == 0:
                if msg == "skip-existing":
                    skipped += 1
                    print(f"[SKIP] {out_path.name}")
                else:
                    ok += 1
                    encode_timings.append((out_path.stem, dt))
                    print(f"[OK]   {out_path.name} ({dt:.3f} s)")
            else:
                fail += 1
                print(f"[ERR]  {out_path.name} -> rc={rc}\n{msg}\n", file=sys.stderr)

    print(f"\n[SUMMARY] 성공 {ok}  건너뜀 {skipped}  실패 {fail}  (총 {len(ply_files)})")
    if log_path is not None and encode_timings:
        encode_timings.sort(key=lambda x: int(x[0].split('_')[-1]) if x[0].split('_')[-1].isdigit() else x[0])
        with log_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['name', 'encode_s'])
            for name, dur in encode_timings:
                writer.writerow([name, dur])
        print(f"[LOG] encode timings -> {log_path}")
    if fail > 0:
        sys.exit(2)

if __name__ == "__main__":
    main()
