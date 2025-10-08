# draco-ros2-roundtrip-git

Draco(구글의 3D 압축 라이브러리)를 이용해 LiDAR 포인트클라우드를 스트리밍하고 복원 품질을 검증하는 ROS 2 워크스페이스입니다. rosbag에 담긴 포인트클라우드를 메모리에서 바로 PLY 스트림으로 변환 → Draco로 압축 → TCP를 통해 서버에 전송 → 복원된 포인트클라우드를 다시 ROS 토픽으로 재생하는 전체 라운드트립 파이프라인을 제공합니다.

## 주요 구성 요소
- `draco_roundtrip`
  - `nodes/stream_client.py`, `nodes/stream_server.py`: bag 재생→인코딩→송신→수신을 큐/스레드로 비동기 처리하는 최신 파이프라인과 임시 파일 자동 정리를 제공합니다.
  - `cli/stream_client.py`, `cli/stream_server.py`, `cli/{monitor,replay}.py`: 동일한 로직을 `ros2 run`과 순수 Python CLI에서 공통으로 사용합니다.
  - `tools/monitor.py`, `tools/replay.py`: RViz 비교 및 프레임 metrics 출력. 루트 `scripts/*.py` 는 모두 이 CLI를 래핑합니다.
- `draco_tools`
  - `core/encoder.py`: Draco 인코딩을 단일 함수로 노출해 스트리밍 클라이언트/배치 인코더/오프라인 파이프라인이 재사용합니다.
  - `cli/encode_ply_to_draco.py`, `bag_to_ply.py`, `offline_pipeline.py`: 공용 코어를 호출하는 CLI 유틸. `bag_to_ply`는 `--stream-fd`를 받아 디스크 없이 PLY를 즉시 스트리밍할 수 있습니다.
  - `analysis/quality.py`: Chamfer-like 지표 계산을 `draco_roundtrip.utils` 의 metrics 와 공유합니다.
- `slam_stream_bridge`: SLAM 실험과 연동할 수 있는 런치 파일 모음

## 사전 준비
1. **ROS 2**: Humble(권장) 또는 호환 버전 설치 후 `source /opt/ros/<distro>/setup.bash` 로 환경을 불러옵니다.
2. **Draco 바이너리**: 공식 릴리스에서 `draco_encoder`, `draco_decoder`를 받아 PATH 에 추가하거나 아래처럼 환경변수를 설정합니다.
   ```bash
   export DRACO_HOME=/path/to/draco/build
   export PATH="$DRACO_HOME:$PATH"
   export DRACO_ENCODER=$DRACO_HOME/draco_encoder
   export DRACO_DECODER=$DRACO_HOME/draco_decoder
   ```
3. **Python 의존성**: `numpy`, `plyfile`, `scipy`, `open3d` 등이 필요합니다. 시스템 패키지 또는 `pip install numpy plyfile scipy open3d`로 설치하세요.
4. **데이터**: 테스트 rosbag을 `data/bags/` 아래에 배치합니다. (예시: `data/bags/rosbag2_2024_09_24-14_28_57/`)

## 빌드 절차
```bash
source /opt/ros/<distro>/setup.bash         # 예: humble, iron
cd /home/kkit/draco-ros2-roundtrip-git/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
`~/.bashrc` 에 위 두 개의 `source` 명령과 PATH 설정을 추가해 두면 새 터미널에서도 바로 `ros2 run` 명령을 사용할 수 있습니다.

## 실행 방법
### 1. 서버
```bash
ros2 run draco_roundtrip stream_server --port 5000
```
`draco_decoder` 가 PATH에 없다면 `--decoder /absolute/path/to/draco_decoder` 로 직접 지정합니다. 서버는 디코딩을 마친 뒤 즉시 임시 `.drc`/`.decoded.ply` 파일을 삭제해 작업 디렉터리를 깨끗하게 유지합니다.

### 2. 클라이언트
다른 터미널에서 다음과 같이 실행합니다.
```bash
ros2 run draco_roundtrip stream_client \
    --bag data/bags/rosbag2_2024_09_24-14_28_57 \
    --topic /sensing/lidar/top/pointcloud \
    --prefix cycle_sample \
    --encoder-workers 0 \
    --max-pending 16
```
- rosbag 재생 시 `configs/qos_override.yaml`을 자동으로 적용해 QoS를 Best Effort로 낮춰줍니다.
- 기본 `--idle-timeout` 은 10초로 설정되어 있어, 초기 로딩 동안 메시지를 기다릴 수 있습니다.
- `--encoder` 옵션을 통해 `draco_encoder` 경로를 직접 지정할 수 있고, `--cl`, `--qp`, `--qg`를 통해 압축 품질을 조정할 수 있습니다.
- `--encoder-workers` 를 생략하면 CPU 코어 수에서 1개를 남기고 자동으로 병렬 인코딩 스레드가 생성됩니다. `--max-pending` 으로 인코딩 대기 큐 길이를 제한할 수 있습니다.

비동기 파이프라인 덕분에 rosbag 재생 속도와 서버 처리 속도가 달라도 프레임이 쌓이지 않고 순차적으로 스트리밍됩니다. 모든 프레임을 처리하고 나면 `data/decoded_from_server/` 에서 생성된 파일은 자동으로 삭제됩니다. RViz에서 `stream_pair/source`, `stream_pair/decoded` 를 동시에 시각화하면 복원 품질을 확인할 수 있습니다.

### 3. 보조 유틸리티
- PLY 생성만 필요한 경우:
  ```bash
  ros2 run draco_tools bag_to_ply --topic /sensing/lidar/top/pointcloud --out data/ply --best-effort
  ```
- SLAM 연동 런치 예시:
  ```bash
  ros2 launch slam_stream_bridge hdl_graph_slam_stream.launch.py
  ```

각 노드/스크립트는 `--help` 옵션으로 세부 인자를 확인할 수 있습니다.

## 디렉터리 구조
- 루트에서 `tree -L 2` 실행 결과
  ```bash
  $ tree -L 2
  .
  ├── configs
  │   ├── client.profile.yaml
  │   ├── draco.json
  │   ├── hdl_graph_slam_stream.yaml
  │   ├── netem.profiles.yaml
  │   ├── qos_override.yaml
  │   ├── ros_topics.yaml
  │   ├── rtabmap_stream.yaml
  │   └── server.profile.yaml
  ├── data
  │   ├── bags
  │   ├── bags_keep
  │   ├── client_tmp
  │   ├── decoded_from_server
  │   ├── ply_stream
  │   ├── results
  │   └── server_tmp
  ├── docs
  │   ├── 3d_slam_setup.md
  │   ├── apps_legacy
  │   ├── HOWTO.md
  │   ├── results_template.md
  │   └── webui
  ├── draco-ros2-roundtrip
  │   ├── analysis
  │   ├── apps
  │   ├── configs
  │   ├── data
  │   ├── docs
  │   ├── launch
  │   ├── logs
  │   ├── Makefile
  │   ├── offline_pipeline.py
  │   ├── pcd_diff.log
  │   ├── qos_override.yaml
  │   ├── README.md
  │   ├── requirements.txt
  │   ├── scripts
  │   └── webui
  ├── logs
  │   ├── ros
  │   └── runs
  ├── migrate_refactor.sh
  ├── README.md
  ├── README.md.keep
  ├── refac.md
  ├── ros2_ws
      ├── build
      ├── install
      ├── log
      └── src

  
  32 directories, 26 files
  ```

- 주요 하위 디렉터리 샘플
  ```bash
  $ tree data/bags -L 2
  data/bags
  ├── rosbag2_2024_09_24-14_28_57
  │   ├── metadata.yaml
  │   └── rosbag2_2024_09_24-14_28_57_0.db3
  └── rosbag2_2024_09_24-14_30_22
      ├── metadata.yaml
      └── rosbag2_2024_09_24-14_30_22_0.db3
  
  2 directories, 4 files
  
  $ tree ros2_ws/src -L 2
  ros2_ws/src
  ├── draco_roundtrip
  │   ├── draco_roundtrip
  │   ├── package.xml
  │   ├── resource
  │   ├── setup.cfg
  │   └── setup.py
  ├── draco_tools
  │   ├── draco_tools
  │   ├── package.xml
  │   ├── resource
  │   ├── setup.cfg
  │   └── setup.py
  └── slam_stream_bridge
      ├── package.xml
      ├── resource
      ├── setup.cfg
      ├── setup.py
      └── slam_stream_bridge
  
  9 directories, 9 files
  ```

- `data/`: rosbag, 인코딩된 `.drc`, 복원된 `.ply` 등 실험 산출물이 위치합니다. `.gitignore` 대상이므로 자유롭게 사용 가능합니다.
- 스트리밍 클라이언트/서버는 실행이 끝나면 임시 `.drc`/`.decoded.ply` 파일을 자동으로 삭제하므로 디렉터리를 수동으로 정리할 필요가 없습니다.
- `logs/`: 실행 로그 저장 위치. 필요 시 비우고 다시 사용하세요.
- `ros2_ws/`: ROS 2 패키지 소스 및 빌드 아티팩트가 있는 워크스페이스 루트입니다.

## 문제 해결
- **QoS mismatch**로 메시지가 수신되지 않을 경우, 클라이언트를 `--reliable` 로 실행하거나 rosbag을 재생할 때 다른 QoS 설정을 사용해 보세요.
- 스트림 파이프가 조기에 끊기면 클라이언트가 즉시 종료되므로, 서버/클라이언트 로그에서 `[CLIENT] WARN: missing context` 등이 없는지 확인합니다.
- `.ros/log` 가 가득 차 Permission 오류가 발생하면 `rm -rf ~/.ros/log/*` 로 정리한 뒤 다시 실행합니다.
- Draco 실행 파일을 찾지 못하면 PATH 또는 환경 변수를 재확인하세요 (`which draco_encoder` 로 확인 가능).

## 참고
- 프로젝트의 모든 스크립트는 ASCII 기반이며, ROS 2 패키지는 `colcon build --symlink-install`로 동적 링크된 상태를 유지합니다.
- 버그나 개선 사항은 Issue/PR로 남겨 주세요.
