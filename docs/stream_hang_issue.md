# Draco 스트리밍 15번째 프레임 중단 이슈 대응 기록

## 현상 요약
- `ros2 run draco_roundtrip stream_client`와 `stream_server` 조합으로 테스트할 때, 약 15번째 프레임 전송 이후 송수신 스레드가 모두 진행되지 않고 대기 상태에 머무는 현상이 반복 발생함.
- 클라이언트/서버 프로세스 자체는 종료되지 않으며, 재시작 전까지 추가 프레임이 처리되지 않음.

## 대응 절차 개요
1. `ros2 run draco_roundtrip stream_monitor`로 모니터링 프로그램 기동
2. 별도 터미널에서 서버(`stream_server`), 클라이언트(`stream_client`) 순으로 실행
3. 모니터 출력에서 파이프라인 단계별 시퀀스 진행 상황과 경고 메시지 확인

> 모니터는 기본적으로 `127.0.0.1:55050`에서 텔레메트리를 수신한다. 필요 시
> `stream_monitor --port ...` 또는 `DRACO_MONITOR_ADDR=host:port` 환경변수를
> 설정해 포트를 맞춰준다.

## 원인 제거 체크리스트
- [ ] **입력 데이터 검증**
  - [ ] 문제 지점 전후 PLY 프레임에 손상/비정상 값이 있는지 검사
  - [ ] 15번째 프레임에 대응하는 rosbag 메시지의 메타데이터(QoS, timestamp) 확인
- [ ] **클라이언트 측 파이프라인**
  - [ ] `ply_stream_reader`가 15번째 이후에도 정상적으로 다음 헤더를 읽는지 로깅 (`ros2_ws/src/.../nodes/stream_client.py`)
  - [ ] `encode_queue` / `send_queue` / `context_store` 크기를 주기적으로 출력하여 병목 위치 확인 *(텔레메트리 이벤트에서 확인 가능)*
  - [ ] `PipelineBudget` 세마포어 사용량 모니터링 (예약 후 미해제 케이스 탐지) *(텔레메트리 이벤트에서 확인 가능)*
  - [ ] `sender_loop`에서 `pending` 딕셔너리에 남는 항목이 없는지 검사
  - [ ] `receiver_loop`가 `pending_decoded`에서 순서 기다림으로 멈추지 않는지 확인
- [ ] **서버 측 파이프라인**
  - [ ] `decode_queue` maxsize (`worker_count * 2`) 초과 여부 확인 *(텔레메트리 이벤트에서 확인 가능)*
  - [ ] 디코더 워커 로그 추가 후 예외/지연 여부 검사 *(텔레메트리 이벤트에서 확인 가능)*
  - [ ] 서버에서 15번째 프레임을 수신하고 응답을 생성했는지 확인 (`seq` 로그) *(텔레메트리 이벤트에서 확인 가능)*
- [ ] **네트워크/프로세스 환경**
  - [ ] TCP 소켓 상태 (FIN/TIME_WAIT 등) 캡처
  - [ ] 네트워크 패킷 드롭/지연 모니터링 (예: `tcpdump`, `ss`)
  - [ ] 시스템 자원(CPU/RAM/디스크 I/O) 사용량 스파이크 점검
- [ ] **외부 프로세스**
  - [ ] `bag_to_ply`가 해당 시점 이후에도 정상 출력 중인지 확인
  - [ ] ROS 2 `rosbag play` 프로세스가 멈추지 않았는지 확인

## 테스트 환경 메모
- 실행 방법: ROS 2 (Humble 기준)에서 `ros2 run draco_roundtrip stream_client`, `ros2 run draco_roundtrip stream_server`.
- rosbag → PLY 변환 파이프라인을 동시 실행( `bag_to_ply` 서브프로세스 포함).
- 기본 QoS 설정: `draco_roundtrip/configs/qos_override.yaml`.

## 시도 내역 및 결과
시도 순서|내용|수정 위치|결과
---|---|---|---
1|인코딩 실패가 발생하면 즉시 파이프라인이 종료하도록 `encode_worker`에서 `stop_event` 및 `error_queue`를 트리거|`ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_client.py` (`encode_worker`)|인코딩 오류가 빠르게 감지되도록 개선되었으나, 15번째 프레임에서의 정지 문제는 그대로 발생
2|송수신 예외 시 소켓을 강제로 종료하고, `recv_message` 루프에 타임아웃을 추가하여 블로킹 방지|`ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_server.py` 및 `draco-ros2-roundtrip/scripts/stream_server.py`|서버 측 블로킹은 완화되었으나 중단 현상은 계속 재현
3|컨텍스트 락 및 순서 보장 로직을 검토하여 데드락 가능성 분석 (코드 리뷰 수준)|`ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_client.py`|락 중첩 조건은 발견되지 않았고, 수정 없이 분석만 진행

## 향후 조사 아이디어 (중복 방지용)
- 15번째 프레임 직전/직후의 큐 상태(`encode_queue`, `send_queue`, `context_store`)를 로깅하여 병목 지점을 명확히 파악
- 서버 측 `decode_queue` 포화 여부와 워커 상태를 모니터링하기 위한 진단 로그 추가
- 네트워크 레벨 패킷 캡처 또는 `strace`를 활용한 소켓 상태 점검
- 특정 프레임 데이터 자체 문제(예: 손상된 PLY) 가능성 검토

> 위 표에 기록된 항목은 이미 수행되었으므로, 추가 해결 방안 수립 시 동일 작업을 반복하지 않도록 참고한다.
