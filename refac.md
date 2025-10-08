Developer: Role: Systems Architect

아래는 리팩토링 제안입니다.

Begin with a concise checklist (3-7 bullets) of what you will do; keep items conceptual, not implementation-level.

# Step 1 — 프로젝트 탑다운(핵심 데이터 흐름) 한눈에
- **rosbag/라이브 LiDAR** → `bag_to_ply.py`가 PLY로 저장 → `stream_client*.py`가 PLY를 **Draco(.drc)**로 인코딩 후 TCP로 전송 → `stream_server.py`가 **Decode**해 PLY로 되돌림 → (옵션) RViz 재생/품질분석/SLAM 연계(HDL Graph SLAM 또는 RTAB-Map).
- **오프라인 일괄처리**는 `offline_pipeline.py` 단일 엔트리로 **저장→인코딩→디코딩→품질분석**을 자동화.

# Step 2 — 파일명 ↔ 역할 매핑 (핵심 코드 우선)

## A. 스트리밍 · 재생 · 모니터링
- `scripts/stream_client.py`  
  rosbag 재생+PLY 저장을 트리거하고, 저장된 PLY를 **draco_encoder**로 인코딩해 서버로 전송/응답 파일 저장(대역폭 요약 포함).
- `scripts/stream_client_replay.py`  
  위와 동일 흐름 + **즉시 재생(원본/복원 포인트클라우드 송출 ROS 토픽으로 퍼블리시)** + 프레임별 간단한 오차 통계(centroid Δ, bbox Δ, 샘플 기반 거리). RViz에서 송 수 비교 제공.
- `scripts/stream_server.py`  
  TCP length-prefixed 프로토콜로 .drc 수신 → **draco_decoder**로 PLY 복원 → 다시 전송(대역폭 요약).
- `scripts/replay_ply_pairs.py`  
  **원본 vs 디코드** PLY 파일 포인트클라우드를 시간축으로 흘리며 두 토픽(`/.../source`, `/.../decoded`)으로 퍼블리시. RViz 중첩 확인용.
- `scripts/replay_with_monitor.py`  
  위 송 재생+각 프레임별 **수치 비교(샘플링, 양방향 KD 조회 or 간이 매칭)**를 콘솔에 리포트.
- `scripts/compare_ply_pair.py`  
  단일 **원본/복원 PLY 파일 1쌍**을 지속 퍼블리시(속 간단).
- `scripts/replay_decoded_ply.py`  
  **복원 PLY들만** 순차 퍼블리시(루프 옵션 있음).
- `scripts/pcd_diff_monitor.py`  
  실시간 **PointCloud2 두 토픽 간** 타임스탬프 매칭 후(지연 허용) 샘플링·ICP(옵션)로 평균/최대 거리, centroid/bbox Δ 등 프레임별 리포팅. 스트리밍 실험 시 현장 지표.

## B. 데이터 변환 · 배치 파이프라인
- `data/bag_to_ply.py`  
  PointCloud2 → (Open3D 우선, plyfile 폴백) **바이너리 PLY 저장**, QoS(BEST_EFFORT 기본/RELIABLE 선택), `--every/--max-frames/--idle-timeout-sec/--voxel-size` 등. 저장 시간 CSV로로그.
- `data/encode_ply_to_draco.py`  
  PLY 배치 인코딩(**원본 stem 보존**, 병렬 workers, skip-existing, 추가 인자 pass-through), 인코딩 시간 CSV로로그.
- `offline_pipeline.py`  
  **원샷 오프라인 파이프라인**: rosbag play → PLY 저장 로그 → Draco 인코딩 로그 → (옵션) 디코드/품질 계산(`analysis/analyze_draco_quality.py`) → **프레임별 사이클 시간 집계 CSV**까지 생성. 종료/정리 신호·환경변수·fast preset 등 현장성 보강.

## C. 품질/성능 분석
- `analysis/analyze_draco_quality.py`  
  DRC **병렬 디코드** → 원본/복원 PLY 페어 매칭 → **KDTree 기반 양방향 최근접 통계(Chamfer-like)**, 임계치 통과율, 압축비율, 디코드 FPS 계산 → **CSV+요약 MD** 생성(임시 PLY 유지/삭제 옵션, tqdm 진행바, 전처리 옵션).

## D. SLAM 연계(3D 맵)
- `configs/hdl_graph_slam_stream.yaml` / `launch/hdl_graph_slam_stream.launch.py`  
  입력 `/stream_pair/decoded`, 프레임/voxel/NDT 설정 포함, 단일 노드 런치.
- `configs/rtabmap_stream.yaml` / `launch/rtabmap_stream.launch.py`  
  RTAB-Map+ICP Odom 2노드 구성, 동일 입력 remap 가능, 매개변수 세트 제공.
- `docs/3d_slam_setup.md`  
  의존성 설치→런치→RViz→맵 저장까지의 절차와 팁.

## E. QoS/설정/웹·앱/메타
- `qos_override.yaml` — 특정 토픽 QoS 덮어쓰기(BEST_EFFORT/keep_last/깊이 10).
- `configs/{client,server}.profile.yaml`, `configs/{netem.profiles.yaml,ros_topics.yaml,draco.json}` — 현재 컨텍스트에서는 템플릿/placeholder 상태.
- `docs/{HOWTO.md,results_template.md}` — placeholder.
- `apps/{run_client.py,run_server.py,summarize_run.py}` — placeholder.(런처/런퍼 역할 계획 파일로 추정)
- `webui/app.py` — placeholder.(후속 대시보드/요약 시각화 자리)
- 최상위/패키지 `README.md`, `requirements.txt` — 현재 내용 없음(채워야 함).
- `Makefile` — 트리상 존재(세부 내용 미제공 → 리팩토링 시 병행 정비 권장).

# Step 3 — 중복·공통화 포인트 (문제형식으로 정리)
1. **PLY 로딩/샘플링/통계 코드가 여러 스크립트에 분산**→ `_load_xyz`, 샘플/통계 루틴이 `replay_*`, `compare_*`, `pcd_diff_monitor.py` 등에 반복. **공용 모듈로 승격 필요**.
2. **클라이언트 2종(재생 유/무) 분기**→ `stream_client_replay.py`가 더 상위 슈퍼셋(재생 on/off 플래그로 단일화 가능).
3. **프로토콜은 안정적(길이+이름 프레임)**이나 메시지 타입/에러 경로가 **문자열 prefix 기반 ('decode-error:...')**. → 간단하지만, **타입 필드/상태코드**를 가진 구조화 메시지로 확장 여지.
4. **설정 산재**(YAML, JSON, CLI): 공통 파라미터 스키마와 기본값 일원화 필요. (예: QoS/프레임명/토픽 접두어/Draco 파라미터)
5. **문서/런처/앱**은 placeholder가 많아, onboarding 난이도↑ (README/HOWTO/requirements/apps/* 보완 필요).

# Step 4 — 권장 리팩토링 구조(러닝커브↓, 재사용↑)

```
draco_roundtrip/                      ← pip/ROS2 가능 파이썬 패키지
  __init__.py
  io/
    bag_to_ply.py                     ← 그대로 이동(함수화)
    ply_codec.py                      ← PLY load/save 유틸(중복 함수)
  draco/
    encode.py                         ← encode_ply_to_draco 함수화
  net/
    protocol.py                       ← 길이프레임 + 타입/상태코드 정의
    server.py                         ← stream_server 로직 이식
    client.py                         ← stream_client(_replay) 통합
  ros/
    publish.py                        ← 공통 퍼블리시/재생(토픽 prefix, QoS)
    monitor.py                        ← pcd_diff_monitor 기능 이식
  analysis/
    quality.py                        ← analyze_draco_quality 함수화
    metrics.py                        ← KDTree/ICP/샘플링 공통화
  cli/                                ← 콘솔 엔트리(선호: Typer/argparse)
    stream_client.py                  ← --play 옵션으로 재생 토글
    stream_server.py
    replay_pairs.py
    monitor.py
    offline_pipeline.py               ← 파이프라인 그대로 제공
  configs/                            ← 현 configs/* 유지 + schema.json
  launch/                             ← 기존 launch/* 유지
  docs/                               ← 3d_slam_setup.md 보강
  webui/
    app.py                            ← 결과/요약 뷰 등 (추후)
```

**핵심 요지**
- "스크립트 모음" → **모듈화된 라이브러리 + 얇은 CLI**로 전환.
- 로딩/통계/퍼블리시/프로토콜/인코더 호출을 **단일 책임 모듈**로 갱신, 중복 제거.
- `stream_client_replay.py`를 기본으로 삼아 `--play/--metrics` 플래그로 **기능 토글**; 단일 바이너리로 현장 운용 단순화.

# Step 5 — 마이그레이션(작업 순서 제안)
1. **공통 유틸 먼저 분리**: `ply_codec.py`, `metrics.py`, `publish.py` 작성 → 기존 스크립트가 이 모듈을 import 하게 수정(기능 동일).
2. **클라이언트 통합**: `client.py`로 로직 이관 후, `cli/stream_client.py`에서 `--play` 유무로 재생 제어. (현 사용 명령과 옵션 이름 유지/alias 제공)
3. **프로토콜 타입화**: `protocol.py`에 `MSG_DATA/MSG_ERR/MSG_EOF` 등 상수·구조체 정의 후 server/client 양쪽 적용. ('decode-error:...' 문자열을 **MSG_ERR(code,msg)**로 치환)
4. **품질분석 라이브러리화**: `analysis/quality.py`로 알고리즘/직렬화 분리 → CLI는 얇게 유지.
5. **문서·런처·요건 정리**: `README.md`에 **3가지 사용 시나리오(온라인/오프라인/SLAM)** 빠른 시작 추가, `requirements.txt` 정비(Open3D/plyfile/scipy/tqdm 등 선택/필수 구분).

# Step 6 — 운용/성능/테스트 개선 팁
- **성능 프로파일 일원화**: `offline_pipeline.py --fast-preset`의 기본값을 **프로파일 YAML**(예: `configs/draco.json`)로 끌어올려 서버/클라/배치 동일 파라미터 체계 구축.
- **테스트 픽스처**: `tests/`에 소형 PLY/DRC 2~3쌍과 **golden CSV/MD** 포함 → CI에서 encode/decode/quality 회귀 검증.
- **QoS 정책 통일**: `qos_override.yaml`을 런치에 주입하거나 ROS 파라미터로 노출해 실험별 변경을 명시화.
- **SLAM 파이프라인 번들 런치**: 서버→클라→SLAM→RViz를 하나의 `bringup.launch.py`로 묶어 "원클릭 실험" 지원. (RTAB-Map/HDL 토글 인자)

---

If you perform any actions (such as code edits or tool invocations), state the purpose and minimal inputs beforehand. After each tool call or code edit, validate the result in 1-2 lines and proceed or self-correct if validation is needed. Set reasoning_effort = medium to match the complexity of this architectural planning and refactoring context. Attempt a first pass autonomously unless missing critical information; if success criteria cannot be met, stop and ask for clarification.