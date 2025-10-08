Developer: Role: Systems Architect

아래는 리팩토링 제안입니다.

Begin with a concise checklist (3-7 bullets) of what you will do; keep items conceptual, not implementation-level.

- 현 `ros2_ws/src` 패키지(draco_roundtrip, draco_tools, slam_stream_bridge) 기준으로 rosbag→Draco 데이터 흐름과 책임을 재정리합니다.
- 노드·도구·분석 스크립트에 흩어진 TCP 프로토콜, PLY 로딩, metrics 계산 로직을 공용 유틸로 승격하는 전략을 제시합니다.
- 루트 `draco-ros2-roundtrip/` 레거시 스크립트를 ROS 2 패키지 기반으로 통합/단순화하는 마이그레이션 경로를 설계합니다.
- SLAM 연계, 설정 일원화, 테스트·문서 자동화를 위한 후속 작업 로드맵을 정리합니다.

# Step 1 — 프로젝트 탑다운(핵심 데이터 흐름) 한눈에
- rosbag/라이브 LiDAR → `ros2_ws/src/draco_tools/draco_tools/bag_to_ply.py`가 `data/ply_stream/` 등에 PLY 프레임을 기록 → `ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_client.py`가 PLY를 Draco(.drc)로 인코딩하고 TCP로 전송, 동시에 ROS 토픽(`/stream_pair/source`, `/stream_pair/decoded`)을 퍼블리시하며 품질 지표를 로그로 남김.
- 서버 측은 `ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_server.py`가 `.drc`를 수신 후 디코딩하여 `data/decoded_from_server/`로 저장하고 다시 클라이언트에 회신.
- 재생/모니터링은 `ros2_ws/src/draco_roundtrip/draco_roundtrip/tools/replay.py`와 `ros2_ws/src/draco_roundtrip/draco_roundtrip/tools/monitor.py`가 원본·복원 PLY를 RViz 토픽과 콘솔 지표로 노출.
- 오프라인 배치는 `ros2_ws/src/draco_tools/draco_tools/offline_pipeline.py`가 rosbag → PLY → Draco → 품질 분석(`draco_tools/analysis/analyze_draco_quality.py`)을 한 번에 수행하고 CSV/MD 리포트를 만든 뒤 SLAM 실험은 `ros2_ws/src/slam_stream_bridge` 런치가 스트리밍 결과를 SLAM 패키지로 연결.

# Step 2 — 파일명 ↔ 역할 매핑 (현 구조와 우선순위)
## A. ROS 2 스트리밍 노드 · 모니터링 (`ros2_ws/src/draco_roundtrip/draco_roundtrip`)
- `nodes/stream_client.py`: rosbag 플레이 제어, PLY 인코딩, TCP 송수신, ROS 토픽 퍼블리시, 간이 metrics 출력, QoS override 탐색(`resolve_qos_override`). 현재 단일 엔트리이므로 기능 토글/공용화에 핵심.
- `nodes/stream_server.py`: TCP length-prefixed 프로토콜 구현, Draco 디코드, 대역폭 요약 로그.
- `tools/replay.py`, `tools/monitor.py`: PLY 페어 재생 및 KDTree 기반 프레임별 metrics 계산, RViz 비교 토픽 출력.
- `data/bag_to_ply.py`: `draco_tools.bag_to_ply`를 호출하는 thin wrapper; ROS 2 패키지에서 재사용하기 위한 연결 지점.
- `configs/qos_override.yaml`: bag 재생 시 QoS 덮어쓰기 기본값.
- `legacy/*.py`: 기존 standalone 스크립트를 보존하는 호환 레이어(직접 실행은 가능하나, 새로운 구조에 흡수 예정).

## B. 파이프라인 공용 툴 (`ros2_ws/src/draco_tools/draco_tools`)
- `bag_to_ply.py`, `encode_ply_to_draco.py`: CLI/라이브러리 겸용 PLY 저장·배치 인코딩 유틸, CSV 로그 생성.
- `offline_pipeline.py`: bag 재생→저장→인코드→품질 분석→cycle time 집계까지 자동화, 프로세스 종료 시그널/정리 로직 포함.
- `analysis/analyze_draco_quality.py`: PLY↔DRC 품질·성능 지표(Chamfer-like) 계산, 다중 worker, 통계 전처리 옵션 제공.

## C. SLAM 연계 (`ros2_ws/src/slam_stream_bridge`)
- `slam_stream_bridge/launch/*.launch.py`: `draco_roundtrip` 토픽(`/stream_pair/decoded`)을 HDL Graph SLAM, RTAB-Map 등으로 연결하는 런치 구성.
- 향후 QoS 및 토픽 remap 일관성을 위해 `configs/*.yaml`과 함께 유지.

## D. 루트 레거시 트리 (`draco-ros2-roundtrip/`)
- `scripts/*.py`와 `analysis/`, `offline_pipeline.py` 등은 ROS 2 패키지 이전의 구현이 대부분이며, 현재 패키지 버전과 코드 중복이 큼. `migrate_refactor.sh`는 새 구조로의 이전을 돕는 스크립트.
- `scripts/*.sh`, `logs/`, `tmp_ply/`는 실험 운영용 도구로 유지되나, Python 로직은 `ros2_ws/src` 패키지를 그대로 호출하도록 단순화 필요.

## E. 최상위 설정/문서
- `configs/*.yaml`, `README.md`, `docs/3d_slam_setup.md`: 사용자 가이드 및 실험 설정이 산재; 일관된 참조 경로와 예제 업데이트가 필요.
- `ros2_ws/install/...` 이하 symlink-install 환경, `ros2_ws/build`/`log`는 빌드 산출물. refactor 시 `colcon build --symlink-install`을 기본 가정으로 유지.

# Step 3 — 중복·공통화 포인트 (현재 코드 기준)
1. **TCP 프로토콜 & 실행 파일 탐색 중복** — `ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_client.py:49`와 `ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_server.py:14` 모두 `find_exe`, `send_message`, `recv_message`를 별도 정의. 동일한 length-prefixed 프로토콜과 오류 문자열 규약을 `draco_roundtrip/utils/protocol.py`(예: `Message`, `ProtocolError`, `open_connection`)로 통합하고, exe 탐색은 `utils/executable.py`에서 환경변수/경로 힌트를 일원화.
2. **PLY 로딩 · metrics 계산 반복** — `stream_client.py:124`와 `stream_client.py:141`, `tools/monitor.py:34`, `tools/monitor.py:78`, `tools/replay.py:22`가 거의 동일한 `_load_xyz`, Chamfer-like 통계, 샘플링 로직을 복제하고, `draco_tools/analysis/analyze_draco_quality.py:160` 이후에도 유사 계산이 존재. `draco_roundtrip/utils/ply.py` (로딩/샘플링)와 `draco_roundtrip/utils/metrics.py` (KDTree, bbox/centroid/Chamfer 계산)로 묶어 노드·툴·분석이 같은 함수를 사용하도록 조정.
3. **Draco 인코딩 파이프라인 분산** — 실시간 클라이언트의 `encode_ply`(`stream_client.py:110`)와 배치 유틸 `ros2_ws/src/draco_tools/draco_tools/encode_ply_to_draco.py:44`가 옵션 처리·로그 로직까지 유사하나 공유하지 않음. `draco_tools/draco/encoder.py` 같은 모듈을 추가해 옵션 파싱/실행/로깅을 재사용하게 만들고, 실시간 경로·배치 경로 모두 공용 함수를 호출하도록 리팩토링.
4. **QoS/경로 설정 산재** — QoS override 탐색은 `stream_client.py:64`에만 존재하고 다른 CLI는 하드코딩된 경로나 매뉴얼 인자에 의존. `configs/qos_override.yaml`, `configs/*.profile.yaml`, `draco_tools/offline_pipeline.py`의 출력 디렉터리 지정 로직을 `utils/config.py`로 일원화해 ROS 2 런치, CLI, 쉘 스크립트가 동일 규칙을 사용하도록 함.
5. **레거시 스크립트와 ROS 2 패키지 병행 유지** — `draco-ros2-roundtrip/scripts/stream_client.py` 등은 최신 노드와 기능 차이가 있으며 유지보수 대상이 겹침. 새 공용 모듈을 만든 뒤, 레거시 스크립트는 얇은 wrapper로 전환하거나 `legacy/`로 이동해 향후 삭제 경로를 명확히 해야 함.

# Step 4 — 권장 리팩토링 구조 (현 패키지 기반 확장)
```
ros2_ws/src/
  draco_roundtrip/draco_roundtrip/
    nodes/
      stream_client.py
      stream_server.py
    tools/
      replay.py
      monitor.py
    utils/
      __init__.py
      protocol.py        ← TCP, 메시지 타입/오류 핸들링
      executable.py      ← encoder/decoder 탐색, PATH 확장
      ply_io.py          ← Open3D/plyfile 로딩, 샘플링, suffix 정규화
      metrics.py         ← Chamfer-like, bbox, centroid, 요약 문자열
      config.py          ← QoS override, 데이터 디렉터리, 환경 변수
    cli/
      stream_client.py   ← Typer/argparse 기반 CLI, ROS 2 entry 에 재사용
      stream_server.py
      monitor.py
  draco_tools/draco_tools/
    core/
      encoder.py        ← encode_ply 공용 함수
      bag.py            ← bag_to_ply 핵심 로직
    analysis/
      quality.py        ← analyze_draco_quality 내 계산을 라이브러리화
    cli/
      encode_ply_to_draco.py
      offline_pipeline.py
```
- 루트 `draco-ros2-roundtrip/`는 Python 로직을 제거하고, `scripts/*.py`는 새 `cli/` 모듈 import 후 `if __name__ == "__main__"` wrapper만 유지하거나 완전히 삭제.
- `setup.cfg`/`package.xml`에 `console_scripts` 및 `entry_points`를 정의해 `ros2 run`과 순수 Python CLI가 동일한 구현을 호출.
- 공용 utils 작성 시 numpy/open3d/scipy 의존성 가드를 유지하고, optional dep 미보유 시 graceful degrade를 문서화.

# Step 5 — 마이그레이션(우선순위 작업 순서)
1. **공용 utils 생성·적용**: `draco_roundtrip/utils/{protocol.py, executable.py, ply_io.py, metrics.py}`를 도입하고 `stream_client.py`, `stream_server.py`, `tools/{replay,monitor}.py`, `draco_tools/analysis/analyze_draco_quality.py`가 새 모듈을 import 하도록 단계적 수정 (기능 동일성 검증 포함).
2. **Draco 인코더/배치 로직 통합**: `draco_tools/encode_ply_to_draco.py`를 `draco_tools/core/encoder.py`에서 구현하도록 분해하고, `stream_client.py`는 해당 함수 호출 + 결과 로그만 수행하도록 정리. 로그 포맷은 기존 CSV/콘솔 출력과 호환.
3. **QoS·경로 설정 공용화**: `utils/config.py`에서 QoS override 경로, spool/output 디렉터리, 기본 prefix를 관리하도록 만들고, `offline_pipeline.py`, ROS 런치, 쉘 스크립트가 같은 헬퍼를 사용하도록 수정. 설정값은 YAML/json 프로필을 읽을 수 있게 확장.
4. **레거시 스크립트 정리**: `draco-ros2-roundtrip/scripts/*.py`는 새 CLI import 후 argparse 정의만 남기거나, 호환성 유지가 불필요하면 README와 함께 제거. `legacy/` 폴더는 deprecation 안내 주석과 함께 남기고 최종 단계에서 삭제.
5. **빌드/배포 스크립트 업데이트**: `setup.cfg`, `package.xml`, `Makefile`, `migrate_refactor.sh`를 새 디렉터리 구조에 맞게 변경하고, `colcon build` + `pytest` 또는 `colcon test`가 새 모듈을 인식하도록 확인.

# Step 6 — 운용/성능/테스트 개선 포인트
- **테스트 픽스처 & 자동 검증**: `ros2_ws/src/draco_roundtrip/tests/` 아래에 소형 PLY/DRC 페어와 protocol/metrics 단위 테스트를 추가하고, `colcon test`에서 소켓 루프백과 KDTree 없는 환경을 모두 검증.
- **엔드투엔드 샘플 파이프라인**: `tests/e2e_roundtrip.sh` 또는 `pytest` 기반 스크립트를 추가해 bag→stream_client→stream_server→monitor 작업이 1~2 프레임이라도 성공하는지 확인. GitHub Actions/CI에서 headless로 실행할 수 있도록 환경 변수를 문서화.
- **성능 프로파일링 일원화**: `configs/draco.json` 또는 profile yaml을 `utils/config.py`가 읽어 `stream_client`, `encode_ply_to_draco`, `offline_pipeline`이 동일 파라미터 세트를 공유하도록 개선.
- **SLAM 번들 런치 통합**: `slam_stream_bridge`에 `bringup.launch.py`를 추가해 stream_server → stream_client → SLAM → RViz가 한 명령으로 실행되도록 하고, QoS/Topic remap 인자는 launch argument로 노출.
- **문서/README 동기화**: refactor 완료 후 `README.md`, `docs/3d_slam_setup.md`, `docs/HOWTO.md`에 새 CLI 경로, 테스트 방법, 데이터 디렉터리 구조를 반영하고, 루트 README와 패키지 README가 중복 없이 링크 공유하도록 정리.
