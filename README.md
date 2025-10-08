# draco-ros2-roundtrip-git

Draco(구글의 3D 압축 라이브러리)를 이용해 LiDAR 포인트클라우드를 스트리밍하고 복원 품질을 검증하는 ROS 2 워크스페이스입니다. rosbag에 담긴 포인트클라우드를 PLY로 변환 → Draco로 압축 → TCP를 통해 서버에 전송 → 복원된 포인트클라우드를 다시 ROS 토픽으로 재생하는 전체 라운드트립 파이프라인을 제공합니다.

## 주요 구성 요소
- `draco_roundtrip`
  - `stream_server`: TCP로 `.drc` 파일을 받아 디코딩 후 결과 PLY를 돌려주는 서버 노드
  - `stream_client`: rosbag 재생 + PLY 추출 + Draco 인코딩 + 서버 전송 + 복원 결과를 ROS 토픽으로 퍼블리시
  - `tools/monitor`, `tools/replay`: 스트리밍 모니터링 및 오프라인 재생 유틸
- `draco_tools`: `bag_to_ply`, `encode_ply_to_draco` 등 파이프라인에서 재사용되는 스크립트
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
`draco_decoder` 가 PATH에 없다면 `--decoder /absolute/path/to/draco_decoder` 로 직접 지정합니다. 서버는 수신한 Draco 파일을 지정된 임시 디렉터리에 저장하고, 디코딩 결과를 클라이언트에게 다시 전송합니다.

### 2. 클라이언트
다른 터미널에서 다음과 같이 실행합니다.
```bash
ros2 run draco_roundtrip stream_client \
    --bag data/bags/rosbag2_2024_09_24-14_28_57 \
    --topic /sensing/lidar/top/pointcloud \
    --prefix cycle_sample
```
- rosbag 재생 시 `configs/qos_override.yaml`을 자동으로 적용해 QoS를 Best Effort로 낮춰줍니다.
- 기본 `--idle-timeout` 은 10초로 설정되어 있어, 초기 로딩 동안 메시지를 기다릴 수 있습니다.
- `--encoder` 옵션을 통해 `draco_encoder` 경로를 직접 지정할 수 있고, `--cl`, `--qp`, `--qg`를 통해 압축 품질을 조정할 수 있습니다.

클라이언트가 실행되면 `data/ply_stream/`에 생성된 PLY 파일을 인코딩한 뒤 서버로 전송하며, 서버에서 돌려받은 복원 결과는 `data/decoded_from_server/`에 저장되고 동시에 ROS 토픽(`stream_pair/source`, `stream_pair/decoded`)으로 퍼블리시됩니다. RViz에서 두 토픽을 비교하면 복원 품질을 시각적으로 확인할 수 있습니다.

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
  │   ├── build
  │   ├── install
  │   ├── log
  │   └── src
  ├── scripts
  │   ├── ddscycle.sh
  │   ├── netem_apply.sh
  │   ├── run_client.sh
  │   ├── run_server.sh
  │   └── summarize.sh
  └── tmp_ply
  
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
- `logs/`: 실행 로그 저장 위치. 필요 시 비우고 다시 사용하세요.
- `ros2_ws/`: ROS 2 패키지 소스 및 빌드 아티팩트가 있는 워크스페이스 루트입니다.

## 문제 해결
- **QoS mismatch**로 메시지가 수신되지 않을 경우, 클라이언트를 `--reliable` 로 실행하거나 rosbag을 재생할 때 다른 QoS 설정을 사용해 보세요.
- `.ros/log` 가 가득 차 Permission 오류가 발생하면 `rm -rf ~/.ros/log/*` 로 정리한 뒤 다시 실행합니다.
- Draco 실행 파일을 찾지 못하면 PATH 또는 환경 변수를 재확인하세요 (`which draco_encoder` 로 확인 가능).

## 참고
- 프로젝트의 모든 스크립트는 ASCII 기반이며, ROS 2 패키지는 `colcon build --symlink-install`로 동적 링크된 상태를 유지합니다.
- 버그나 개선 사항은 Issue/PR로 남겨 주세요.
