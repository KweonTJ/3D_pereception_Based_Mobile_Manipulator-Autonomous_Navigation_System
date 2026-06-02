# leader_task_manager

리더 로봇의 작업 상태, 파지 상태, 플래투닝 활성 상태를 `/leader/*` 토픽으로 발행하는 패키지다.

## 주요 토픽

- 입력
  - `/mp_control/status`: 리더 파지 제어 상태 문자열
  - `/cargo/events`: 파지/적재 이벤트
  - `/cargo/current_id`: 현재 물체 ID
- 출력
  - `/leader/task_state`
  - `/leader/cargo_state`
  - `/leader/follower_enable`
  - `/leader/platoon_mode`

## 파지 중 상태 처리

실제 리더에서 전면 depth가 47cm 이후 불안정해지면 `mp_control`은 전면 bbox 크기와 EEF bbox를 이용해 근접 파지 단계로 넘어간다. 이때 status에 `close front bbox-size object ... EEF pregrasp`가 포함될 수 있다.

이 상태는 베이스 주행이 아니라 팔 pregrasp/파지 준비 단계이므로 `leader_task_manager`는 이를 `PICKING`으로 분류한다. 이 분류가 빠지면 모니터와 팔로워 제어 쪽에서 리더가 다시 `MOVING`으로 보일 수 있고, 팔을 최대로 뻗는 시점에 리더 주행이 다시 열리는 것처럼 보일 수 있다.

## 플래투닝 모드

- `FOLLOW`: 리더 이동 또는 기본 추종 허용
- `STANDBY`: 파지/적재 중 추종 대기
- `STOP`: 오류 또는 작업 종료

파지 중에는 팔 동작이 우선이므로 `PICKING` 상태를 유지하고, 적재/회전 단계에서는 `STANDBY`를 사용한다.
