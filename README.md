# draco-ros2-roundtrip-git

Draco 기반 포인트클라우드 스트리밍 실험을 ROS 2 패키지로 정리한 워크스페이스입니다.
`colcon` 빌드 후에는 표준 `ros2` 명령으로 노드를 실행할 수 있습니다.

## 구성
- `draco_roundtrip`: 스트리밍 클라이언트/서버 노드 및 모니터링 유틸
- `draco_tools`: bag → PLY 변환, Draco 인코딩 등의 보조 스크립트
- `slam_stream_bridge`: SLAM 스트리밍 실험용 런치 파일 모음

## 준비 및 빌드
```bash
source /opt/ros/<distro>/setup.bash   # 예: humble, iron
cd /home/kkit/draco-ros2-roundtrip-git/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 실행 예시
- 서버: `ros2 run draco_roundtrip stream_server --decoder /path/to/draco_decoder`
- 클라이언트: `ros2 run draco_roundtrip stream_client --bag data/bags/... --topic <PointCloud2 토픽>`
- PLY 저장: `ros2 run draco_tools bag_to_ply --topic /pointcloud --out data/ply`
- 런치: `ros2 launch slam_stream_bridge hdl_graph_slam_stream.launch.py`

추가 옵션은 각각 `--help` 로 확인할 수 있습니다. 대용량 rosbag/결과물은 `.gitignore` 로 관리에서 제외되어 있으므로 필요에 따라 `data/` 폴더에 배치해 사용하세요.
