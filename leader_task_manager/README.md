# leader_task_manager

리더 로봇의 작업 상태, 적재 상태, 플래투닝 모드를 외부 모니터와 팔로워가 이해할 수 있는 상태 토픽으로 정리하는 패키지다.

## 주요 토픽

- 입력: `/mp_control/status`
- 출력: `/leader/task_state`
- 출력: `/leader/cargo_state`
- 출력: `/leader/follower_enable`
- 출력: `/leader/platoon_mode`

## 상태 매핑 기준

`mp_control`은 실제 파지 과정에서 사람이 읽을 수 있는 상태 문자열을 `/mp_control/status`로 낸다. `leader_task_manager`는 이 문자열을 보고 리더의 상위 상태를 다음처럼 정리한다.

- 베이스 접근, depth limit 이후 color triangulation 접근: `MOVING`
- EEF pregrasp, joint pregrasp trajectory 대기, close visual bbox 기반 pregrasp: `PICKING`
- grasp 완료: `WAIT_FOLLOWER`, cargo `GRASPED`, platoon `STANDBY`
- handoff/place/loading: `WAIT_FOLLOWER` 또는 `PLACING_ON_FOLLOWER`
- loaded 완료: `CARGO_LOADED`, cargo `LOADED`, platoon `STANDBY`

## 파지 이후 주행 재개 차단

현재 실제 리더 실험에서는 물체 close/handoff 직후 로봇 베이스나 팔로워가 다시 움직이면 파지 자세가 틀어진다. 그래서 `resume_follow_after_loaded` 기본값을 `false`로 둔다.

- `resume_follow_after_loaded: false`: `LOADED`, `PLACED`, `CARGO_LOADED` 이후 `/leader/platoon_mode=STANDBY`, `/leader/follower_enable=false`를 유지한다.
- `resume_follow_after_loaded: true`: 기존처럼 loaded 이후 `/leader/platoon_mode=FOLLOW`로 복귀한다.

즉 현재 기본 동작은 물체를 잡고 내려놓은 뒤에도 자동 주행을 재개하지 않는다.

`mp_control` handoff 로그는 `placed` 이벤트 직후에도 `handoff release: ...`,
`handoff stay: ...` 상태를 이어서 낸다. 이 상태를 단순 handoff 진행 중으로
처리하면 이미 `LOADED`로 바뀐 상태가 다시 `GRASPED/WAIT_FOLLOWER`로 되돌아가고,
`/leader/follower_enable`도 다시 `true`가 된다. 현재 매핑은 `HANDOFF RELEASE`,
`HANDOFF STAY`, `HANDOFF ... LOADED` 계열 상태를 loaded 완료 상태로 처리해서
close 이후 로봇 주행이 다시 살아나지 않게 한다.

## 2026-06-03 로그 반영

실제 리더 로그에서 `color triangulation unavailable; using close visual bbox for EEF pregrasp...`와 `waiting for joint pregrasp trajectory...`가 파지 단계 중에 반복되었다. 기존 매핑은 일부 문자열을 알 수 없는 상태로 처리해서, 리더 작업 상태가 `PICKING`에서 `MOVING`으로 흔들릴 수 있었다.

현재는 다음 문자열을 파지 단계로 명시한다.

- `COLOR TRIANGULATION UNAVAILABLE`
- `CLOSE VISUAL BBOX`
- `EEF PREGRASP`
- `JOINT PREGRASP TRAJECTORY`

이 상태에서는 팔로워/모니터가 리더를 주행 중으로 오해하지 않도록 `/leader/task_state`를 `PICKING`으로 유지한다.

또한 한 번 `PICKING`에 들어간 뒤에는 `AFTER DEPTH LIMIT`, `COLOR TRIANGULATION APPROACH`, `WAITING FOR BASE APPROACH`, `WAITING FOR FRESH`류 상태가 다시 들어와도 `MOVING`으로 되돌리지 않는다. 최근 로그에서 `MOVING -> PICKING` 직후 `PICKING -> MOVING`으로 빠지며 조인트 pregrasp 로그가 나오지 않았기 때문에, 파지 단계 내부의 대기/근접 문자열은 모두 파지 상태로 유지한다.
