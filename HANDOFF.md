# Grippers Pi — 인수인계 (2026-09-07 갱신, 하드웨어 마지막 날)

> ## ⚠️ 먼저 — 이 팔이 지금 어느 캘리브레이션인지 확인하세요
>
> 2026-08-29 에 VLA 시연 수집을 준비하며 **LeRobot 캘리브레이션이 서보의
> `Homing_Offset` 을 덮어썼습니다.**
>
> ```
> Present_Position = Actual_Position - Homing_Offset
> ```
>
> `floor_grasp_profiles.py` 의 교시 자세는 RAW 서보값이라, 오프셋이 바뀌면
> **같은 숫자가 다른 물리 자세**가 됩니다.
>
> **오프셋은 서보 EEPROM 에 있지 git 에 있지 않습니다.** `git checkout` 으로
> 바뀌지 않습니다. 그래서 두 갈래를 이렇게 나눠 씁니다.
>
> | 하는 일 | 브랜치 | 팔의 캘리브레이션 |
> |---|---|---|
> | 베이스라인 미션 | `kica927/baseline_mission` | **교시 당시** (되돌린 상태) |
> | VLA 시연 수집·추론 | `kica927/smolVLA-version` | **LeRobot 새 캘리브레이션** |
>
> 확인:
> ```
> python3 tools/arm/restore_taught_offsets.py
> ```
>
> 베이스라인으로 되돌리기 (**팔이 중력으로 내려옵니다 — 아래를 비우고**):
> ```
> python3 tools/arm/restore_taught_offsets.py --apply --yes
> ```
>
> `arm_driver_node` 가 기동할 때 이것을 대조하고, 다르면 **기동을
> 거부합니다**(`ArmCalibrationMismatchError`). 경고가 아니라 거부인 이유는
> shoulder_pan 가동폭이 2493 → 2087 로 줄어 있어(차체·라이다에 막힘)
> 어긋난 채 움직이면 부딪히기 때문입니다.
>
> 팔을 다시 교시했다면 `floor_grasp_profiles.TAUGHT_HOMING_OFFSETS` 도 같이
> 갱신하세요. 자세와 오프셋은 한 쌍입니다.

이 파일은 **짧은 진입점**이다. 상세 이력·실측·근거는 `grippers_docs/`
(맥 `~/Desktop/intel/grippers_project/_작업_grippers/grippers_docs` — 2026-09-06
폴더 재정리로 경로가 바뀌었다, 예전 경로 `~/Desktop/intel/grippers_docs`는 더
이상 없다)의 다음 두 문서가 현행이다.

- `grippers_작업정리_20260828.md` — 문서·Pi 저장소·Host 저장소 종합 (먼저 읽을 것)
- `grippers_handover_20260827.md` — 08-27~28 Pi 작업 상세, 실측표, Pi 실행 상태
- `grippers_host_requests_20260827.md` — Host 팀이 고쳐야 할 것 + 번역 코드 초안

이전 버전(2026-08-24)에 있던 `scan_track_return.py`·`auto_approach_grasp_rook.py`·
그리퍼캠 절차는 **2026-08-26 역할 분담 확정으로 전부 삭제됐다.** 그 문서를
근거로 작업하지 말 것.

---

## 0. 우리끼리 약속 (작업 규칙) — 먼저 읽을 것

여러 세션에 걸쳐 사용자가 명시적으로 정한 규칙을 전부 모았다. 일부만 보고
넘어가지 말 것 — 특히 아래 "버전관리"·"안전" 항목은 예전 버전 문서에 없던
것들이라 놓치기 쉽다.

### 접속 · 환경
- **존댓말**: 한국어 응답은 항상 존댓말.
- **Pi 접속**: `ssh pi@192.168.0.7` (mDNS `raspberrypi.local`도 됨).
- **컨테이너 진입 (사람, 대화형)** — 반드시 진짜 TTY에서:
  ```
  cd ~/docker && ./exec_shell.sh
  ```
- **컨테이너 진입 (자동화, 비대화형)**:
  ```
  docker exec IntelPi bash -lc '명령'
  ```
- **셸 방언**: `exec_shell.sh` 세션은 zsh → `setup.zsh`. `bash -lc` 경로는 bash → `setup.bash`.
- **`ROS_DOMAIN_ID=21`** — 컨테이너 안 모든 셸에서 예외 없이 가장 먼저 export.
- **ROS 환경 (bash 경로)**:
  ```
  export ROS_DOMAIN_ID=21
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
  ```
  `peripherals/depth_camera.launch.py`·`controller/odom_publisher.launch.py`는
  `need_compile`, `DEPTH_CAMERA_TYPE=ascamera`, `MACHINE_TYPE=MentorPi_Mecanum`을 export로 넘길 것.
- **경로**: Pi 호스트 `~/docker/shared/grippers` = 컨테이너 `/grippers`. 맥 클론과 별개 클론.
- **bringup/teardown은 반드시 `tools/ops/bringup_now.sh` / `stop_bringup.sh` 로만** —
  `docker exec bash -lc`로 launch 파일을 직접 때리지 않는다. `bringup.log`는
  이제(2026-09-07부터) 컨테이너 밖 호스트 `/home/pi/docker/shared/bringup_logs/`
  에도 실시간 미러링된다 — 컨테이너가 재기동돼도 로그가 살아남는다.
- **배포**: `domain/`·`tools/`는 `git pull`만. `ros2_ws/src/**`는 `colcon build --packages-select <패키지>`
  후 해당 노드를 PID로 골라 재기동 (`pkill -f "ros2 run grippers"`처럼 뭉뚱그리지 말 것).
- **배포·재시작 뒤 `perception_node` 반드시 재기동** (`depth_cam_rotate_node`도 같이).
- **Pi 컨테이너의 시간(`date`, 로그 타임스탬프)은 UTC다.** 사용자에게 보고할 땐
  항상 KST(UTC+9)로 환산해서 말할 것.
- **사용자에게 주는 셸 블록에 `#` 주석 금지** — 그 사람 zsh에 붙여넣으면 깨진다.
- 저장소 `docs/`는 권위 자료로 취급하지 않는다 — 근거가 필요하면 먼저 사용자에게 물을 것.

### 버전관리
- **원격은 `personal-mirror`(kica927/grippers)에만 push.** `origin`(팀 grippers-intel
  저장소)에는 **절대 push·merge·PR 금지** — 이 문서의 예전 버전에 있던 "origin+
  personal-mirror 둘 다 push"는 낡은 지시이니 따르지 말 것.
- 브랜치·PR은 `kica927/` 접두어.
- **새로 만든 브랜치는 사용자가 명시적으로 승인하기 전까지 절대 병합하지 않는다**
  — 같은 세션 안에서 빌드·테스트까지 끝냈어도 예외 없음.
- 팀원(`sysy009`, GitHub `sysy009/grippers` 포크)의 브랜치는 **병합하지 않고
  체크아웃을 통째로 스왑**한다 — 두 계보를 그대로 보존.
- **Pi에서 커밋했으면 로컬 맥북(personal-mirror)에도 같이 반영**한다.
- `grippers-baseline-wt` 워크트리에서 작업이 끝나면, 메인 `grippers/` 체크아웃도
  같은 HEAD로 동기화해 둔다.
- `kica927/smolVLA-version`은 **stretch 브랜치** — baseline_mission에 절대 병합 금지.

### 안전 (차량 정지 / 하드웨어)
- **차량이 멈췄다고 topic 값만 보고 판단하지 않는다.** 정지는 `stop=True` 명령
  재전송으로만 시도한다 — **모터에 쓰는 노드를 죽여서 멈추려 하지 않는다.**
  STM32는 마지막으로 받은 명령을 그대로 래치하므로, 노드를 죽이면 그 값이 뭐든
  그대로 굳어버린다.
- ROS2 노드를 `kill -STOP`/`-CONT`로 잠깐 멈추지 않는다 — DDS discovery가
  조용히 깨진다.
- SO-ARM101의 USB(`/dev/soarm`)를 뽑으면 **라즈베리파이 본체 전원도 같이 나간다**
  (전원을 공유함).
- 그리퍼는 최대한 세게 쥐는 쪽을 선호한다 — 살살 쥐는 방향으로 튜닝하지 않는다.
- 팔 경로: 웨이포인트를 거쳐 들어올리고, **하강하기 전에 그리퍼부터 연다** —
  바닥 높이에서 옆으로 스윕하는 경로는 금지.
- STM32 부저는 항상 조용하게 — 공진 주파수를 피하고 아주 짧게만 울린다.
- 팔은 전원이 켜지면 곧바로 IDLE 자세로 자동정렬한다("첫 이동 요청까지 미룬다"는
  코드의 원래 의도를 사용자가 뒤집은 것).
- **`stop_bringup.sh` 등 kill/teardown 계열은 Claude가 직접 실행해도 된다** —
  "kill은 사람이 직접 실행해야 한다"는 이 저장소의 기본 관례(위 스크립트 주석 참고)를
  사용자가 명시적으로 뒤집은 예외다.
- 팀원이 실기 Pi에서 작업 중인 게 확인되면, 그 시간 동안은 SSH·git reset·재빌드
  등 Pi를 건드리는 어떤 것도 하지 않는다 — 로컬 맥 저장소 안에서만 작업한다.

### 캡처 / 로깅
- 실기 bag 녹화는 **RGB(가급적 압축본)+작은 토픽만** 남기고 depth 이미지·
  포인트클라우드는 뺀다.
- `ros2 bag record -a`(전체 토픽)를 절대 그대로 쓰지 않는다 — 58GB 디스크가
  20분 안에 찬다.
- 호스트 쪽 카메라 캘리브레이션 파일(`grippers-host-mac/host/calib/*.npz`)은
  절대 삭제·이동·재생성하지 않는다.

### 인식 / 판정
- OpenCV 오버레이·그리기 색상은 red 계열을 피하고 green/blue/gray 위주로 쓴다.
- 물체까지의 거리는 depth캠이 아니라 **RGB 바운딩박스 픽셀 면적**으로 추정한다.
- 그리퍼캠 기반 파지 자동판정은 폐기됐다 — 그리퍼캠은 **실시간 모니터링 전용**
  (캠 자체는 2026-08-30에 재부착 결정됨). GRASP의 "물체가 사라졌는가" 판정은
  뎁스 카메라가 담당한다.

---

## 1. 지금 상태 (2026-09-07 갱신 — git log·코드·pytest로 재확인)

| 항목 | 상태 |
|---|---|
| 브랜치 | `kica927/baseline_mission` @ `3964ddd` — `personal-mirror` 대비 **0커밋**(완전 동기화), `origin` 대비 64커밋 앞섬(의도된 것 — origin엔 push 안 함) |
| 테스트 | **633개 통과** (`/opt/homebrew/bin/python3.11 -m pytest tests`, 2026-09-07 재확인) |
| Pi 단독 기능 | 파지 → CARRY → 저속 접근 → 자동정지 → INSERT, 여섯 클래스 전부 실기 검증 (08-27) |
| Host ↔ Pi 연동 | 09-02부터 실기로 연동된 것으로 판단(§1a, 08-30판에서 이관, 이번 세션에 새로 재확인한 것은 아님) |
| `use_fake_base` 기본값 | 코드(`mission_orchestrator_node.py:68`)상 `False`(진짜 바퀴) |
| **모터 워치독/시리얼 write_timeout** | **구현 자체는 완료**(STM32측 09-05, SO-ARM101 그리퍼측 09-06/09-07 커밋 완료). 다만 두 값(0.2초·0.5초 등) 전부 **여전히 추측값** — 실기 정상 왕복 지연을 재고 조정하는 일이 남아 있음(아래 §4 참고) |
| **회전정지 감시(IMU 자이로)** | 2026-09-07 신규 추가(`d2eed6b`) — 0속도 재전송 뒤에도 자이로 z축이 크면 경보. 임계값(`ROTATION_STALL_GZ_THRESHOLD_RAD_S` 등) 전부 미실측 |
| **Hailo-10H** | 정상 작동 중(오늘 부팅 확인: `hailortcli identify` 정상, 벤치마크 14.5ms, CPU YOLO 대비 26.7배). 다만 issue #189(2026-08-22 최초 발견)의 **간헐적 하드웨어 고장은 미해결** — 원인은 DDR 손상으로 추정, 소프트웨어로 근본 해결 불가능. CPU YOLO 폴백은 항상 같이 로드되어 있어 재발해도 자동 대체됨 |

### 1a. Host↔Pi 연동 판단 근거와 남은 확인

- `domain/ports/baseline_ports.py`의 `HostCommand`가 이미 확정 5필드(`state`/`linear_x`/`linear_y`/`angular_z`/`stop`) 규격으로 구현돼 있다 — 8/27 요청 문서가 지적한 "확정 이전 규격" 문제가 아니다.
- git log에 09-02 실기, 09-04 밤(toy 입구 밖 투하 사고) 등 **Host 명령으로 로봇이 실제로 움직인 사건**이 날짜별로 기록돼 있다. "투하 사고"는 옮기다 실패한 사건이지 연결이 안 됐다는 뜻이 아니다.
- 다만 이 판단은 로컬 git log·코드 대조로 재구성한 것이고, **지금 이 순간 Pi가 그 상태로 떠 있는지는 최근 세션에서 SSH로 재확인하지 못했다.** 다음 접속 시 컨트롤러→orchestrator 순서로 띄운 뒤 `ros2 topic info /cmd_vel`의 구독자 수로 확인할 것(`RUNBOOK_2026-09-08.md` §3.5 참고).

## 2. 구조 한 줄씩

- `domain/task/baseline_mission.py` — 명령 구동형 FSM `IDLE→APPROACH→GRASP→CARRY→APPROACH_BOX→INSERT→DONE`.
- `domain/task/baseline_constants.py` — 실측/지시 상수. `unresolved()`는 비어 있다.
- `domain/task/motion.py` / `preconditions.py` / `corrections.py` — 속도 클램프, GRASP/INSERT 조건, Host용 `fix`.
- `domain/adapters/real/udp_host_link.py` — Host↔Pi UDP(5005 명령 / 5006 보고).
- `ros2_ws/src/grippers_mission` — `mission_orchestrator_node` (10 Hz 루프).
- `ros2_ws/src/driver/ros_robot_controller/ros_robot_controller_sdk.py` — STM32 시리얼 드라이버.
  모터 워치독 스레드, IMU 자이로 엿보기 캐시(회전정지 감시), `buf_write()` 재연결
  로직이 전부 이 파일에 있다. **벤더 패키지지만 이 프로젝트가 직접 수정해 온
  파일**이라(2026-09-05~07), ruff exclude 목록에 있어도 필요한 수정은 계속 여기 한다.
- `tools/basket_approach_insert_test.py --profile <클래스>` — INSERT 통합 harness.
- `tools/grasp_geometry_calibrate.py --mode k|jaw|load|scale|confirm` — 파지 기하 실측 도구.
- `tools/host_link_conformance.py --as-is|--translated` — Host 실제 코드와 로컬 적합성 시험(하드웨어 불필요).
- `tools/ops/bringup_now.sh` / `stop_bringup.sh` / `test_ready.sh` — bringup 표준 진입점.
- `third_party/soarm_provided_d/` — 서브모듈(제3자 저장소 `th2102da/soarm_provided_d`,
  detached HEAD 관례). 2026-09-07에 `kica927/write-timeout-fix` 브랜치로 write_timeout
  수정을 로컬 커밋했으나 **원격 push는 권한 문제로 보류 상태**(§4 참고).

## 3. 다음 접속 시 순서

1. 컨테이너 `/grippers`에서 `git pull`(personal-mirror) → `3964ddd` 이후 확인.
2. §1a대로 `ros2 topic info /cmd_vel` 구독자 수로 Host↔Pi 연동이 실제로 살아 있는지 확인.
3. `perception_node`·`depth_cam_rotate_node` 확인.
4. **§4의 미검증 항목부터 우선 처리** — 특히 오늘 마지막에 반영한 `BOX_APPROACH_MARGIN_M=0.28`은 배포만 되고 실기로 단 한 번도 못 돌려봤다(카메라 문제로 미션이 중단됨).
5. 물리 상수 실측 3종 중 **`T_stop`은 오늘 오전(09:00경) 완료됐다** — 아래 §4 참고.
   나머지 둘(모터 워치독 발동 시간, `identify_target` 6클래스 왕복 지연)은
   여전히 미완료다. `pi_capture/mac/analyze_stop.py`·`analyze_watchdog.py`로 분석.
6. 사선 진입 INSERT 15°/30° 실측 — `grippers-host-mac/host/manual_insert_probe.py` (WASD 수동 접근, 화면에 `pose.yaw_deg`·`gate.facing_error_deg` 실시간 표시됨 — 추가 개발 불필요, 바로 사용 가능). 이것도 여전히 미완료.

---

## 4. 2026-09-07 (오늘) 요약 — 다음 사람이 반드시 알아야 할 것

### `T_stop` 실측 완료 (오전, `host/t_stop_probe.py`)

`grippers-host-mac/host/logs/tstop_20260907_000217/`(최종)와 `tstop_20260906_232046/`
(그 전 재시도)에 원본 CSV가 남아 있다. `analyze_stop.py`로 뽑은 전체 요약은
정지 이벤트 35건 · T_stop 중앙값 104.4ms(평균 106.9ms, 범위 100.0~201.5ms) ·
오버슈트 중앙값 0.0mm(허용창 ±15mm 안).

⚠️ **이 요약을 그대로 "0.1m/s 5회 + 0.06m/s 5회"로 읽으면 안 된다.** 이벤트를
직전 명령 속도별로 나눠 보면:

| 직전 `linear.x` | 이 값이 나온 이벤트 수 | T_stop 중앙값 |
|---|---|---|
| **0.1** (계획된 값) | 3건 | 100.5ms |
| **0.06** (계획된 값) | 1건 | 106.1ms |
| -0.1 (후진) | 2건 | 102.8ms |
| -0.06 (후진) | 1건 | 106.1ms |
| 0.14 (계획에 없던 값) | 18건 | 106.7ms |
| 0.2 (계획에 없던 값) | 11건 | 101.7ms |

**35건 중 계획된 0.1/0.06 조합은 8건뿐이고, 나머지 29건(0.14·0.2)은 이
녹화 창에 같이 잡힌 다른 주행 명령**으로 보인다 — `t_stop_probe.py`는
0.1과 0.06만 보내는데 그 값들이 로그에 있다. 캡처플랜이 원한 "각 속도
5회"에는 못 미친다(0.1은 3회, 0.06은 1회뿐). 다만 **모든 속도 구간에서
T_stop이 100~110ms 안에 일관되게 몰려 있고 오버슈트가 사실상 0**이라는
결론 자체는 바뀌지 않는다 — 이 지표가 애초에 "정지 명령까지 걸리는
소프트웨어 왕복 지연"을 재는 것이라(속도 자체가 아니라), 속도와 무관하게
비슷하게 나오는 게 자연스럽다. 그래도 표본을 명확히 나눠 5회씩 채우는
재측정이 남아 있다면 그게 더 확실하다.

### 있었던 일

오늘 실기 중 **차량이 정지 명령에 반응하지 않고 계속 도는 사고가 두 번** 났다
(배터리를 뽑아 물리적으로만 해결). 조사 결과, 이 SDK는 STM32로부터 **모터
회전 자체를 되읽어오는 프로토콜이 아예 없다** — `buf_write()`는 오직 쓰기
전용이고, `recv_task`의 패킷 파서 맵에 엔코더/속도 리포트 타입이 없다.
소프트웨어 스택 전체가 "명령을 보냈다"까지만 확인하고 "로봇이 실제로 그
명령대로 됐다"는 **어디서도 검증하지 않는 완전 개루프 구조**라는 게 이
사고로 드러났다 — 이건 이번에 고친 개별 버그가 아니라 **이 하드웨어
플랫폼의 구조적 한계**다.

### 부분 보완 (근본 해결 아님)

1. IMU 자이로를 엿보기 캐시로 저장해 모터 워치독에 결합 — 회전만 감지, 임계값 미실측
2. 미션 종료 시 stop 명령 뒤 카메라로 2초 더 관찰해 실제 정지 확인(Host 쪽) — 종료 시점 한정
3. 모터 워치독: `buf_write()` 쓰기가 실제로 성공했을 때만 타임스탬프 갱신 —
   두 사고 모두 로그상 쓰기 예외가 0건이었어서 이 자체가 원인이었는지는
   확인되지 않았다

**결론: 이 개루프 한계는 다음 사람도 반드시 알고 시작해야 한다.** 소프트웨어를
아무리 고쳐도 "명령이 STM32에 실제로 도달해 실행됐다"를 확인할 방법 자체가
없다.

### 같은 날 났던 별개 사고 — 바구니 접근

체스말을 상자에 넣는 도중 진입각이 클 때 바구니에 부딪히는 사고가 반복됐다.
원인을 세 겹으로 나눠 각각 고쳤다:

1. 팔 SAFE_300 servo1 보정 한계각: 30° → 45° (`fe72178`)
2. Host `basket_target.MAX_FACING_ERROR_DEG`: 팔 한계와 정렬해 45°로 (host-mac `488f832c`)
3. **진짜 근본 원인**: `mission.py`의 `_box_front_xy()`가 로봇이 지금 어느
   각도로 서 있든 무관한 **고정된 목적지 점 하나**만 계산한다 — 차체 회전을
   전혀 고려하지 않는다. `BOX_APPROACH_MARGIN_M`을 0.15→0.20→**0.28**m로
   늘려 각도 무관하게 여유를 키웠다(host-mac `bfc1b5bd`, `36d53f1c`)

⚠️ **0.28m은 배포만 됐고 실기 검증이 전혀 안 됐다** — 반영 직후 다음 미션
실행이 (관련 없는) macOS 카메라 파이프라인 문제로 멈춰서 중단됐다. 다음
사람이 가장 먼저 확인해야 할 것이다. 근본 원인(`_box_front_xy`가 접근각을
아예 안 본다는 점) 자체는 아직도 안 고쳐져 있다 — 여유거리를 늘린 것은
땜빵이다.

### 정리된 것

- Pi 컨테이너 좀비 프로세스 15개 정리, bringup 깨끗하게 재기동 가능한 상태로 종료.
- `bringup.log`가 컨테이너 밖(`/shared/bringup_logs/`)에도 실시간 미러링되도록
  고쳐서, 다음부터는 컨테이너 재시작으로 포렌식 로그가 사라지지 않는다.
- `third_party/soarm_provided_d` 서브모듈에 하루 넘게 커밋 안 된 채 남아있던
  write_timeout 수정(그리퍼/팔 버스에도 STM32와 같은 결함이 있었음)을 발견해
  `kica927/write-timeout-fix` 브랜치로 로컬 커밋. **원격(`th2102da/soarm_provided_d`)
  push는 권한 문제로 실패** — 그 저장소에 쓰기 권한이 있는 사람이 대신 push할 것.

### RoboSec(포트폴리오 보안 프로젝트) — 오늘 실기 재확증

`~/Desktop/intel/grippers_project/robosec`에서 `RUNBOOK_2026-09-08.md` 절차를
하루 앞당겨 오늘 실행했다. F2(IDLE 상태에서 속도 실행됨)와 a3 재전송 효과는
예측대로 재현됐다. F1(nan 클램프)은 예측과 갈렸다 — APPROACH 상태에서는
raw 명령 자체가 모터로 전달되지 않아 클램프 경로를 아예 관측하지 못했다.
**F1의 진짜 실물 회귀 확인(IDLE 상태에서 nan 단독 주입)이 아직 안 끝났다** —
내일(09-08) 마지막 하드웨어 기회에 우선순위로 둘 것
(`robosec/results/onhardware_2026-09-07.md` 참고).

---

## 5. 여러 세션째 계속 미완료인 것 (반복 기록 — 놓치지 말 것)

- ✅ `T_stop`(정지 지연·오버슈트) 실측 — **2026-09-07 오전 완료**(§4 참고).
  다만 계획된 "0.1/0.06 각 5회"에는 못 미치고(각각 3회·1회뿐), 나머지 이벤트는
  계획에 없던 다른 속도(0.14/0.2)가 섞여 있다 — 여유가 있다면 깨끗하게
  5회씩만 다시 재는 게 더 확실하다.
- 모터 워치독 발동 시간 실측 — write_timeout/watchdog 값(0.2s/0.5s)이 여전히 추측값.
- `identify_target` 6클래스 왕복 지연 실측.
- 사선 진입 INSERT 15°/30° 실측(`manual_insert_probe.py`로 바로 가능).
- 포트폴리오 시각자산 7종 중 5종 미확보(로봇 실물 사진, 그리퍼 클로즈업, 전체
  사이클 데모 영상, 작업공간 전경, **실패 장면**) — 하드웨어 없어지면 영원히
  못 찍는다.
