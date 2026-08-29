# tools/vla — SmolVLA 스트레치 버전

> 브랜치 `kica927/smolVLA-version`. **머지 대상이 아닙니다.**
> 베이스라인(규칙 기반 미션)이 정본이고, 이것은 그 위에 얹는 확장입니다.

사람이 텔레옵으로 시연한 것을 정책이 배우고, 그 정책이 **리더 암이 있던
자리에** 들어갑니다. 팔을 움직이는 경로는 바뀌지 않습니다.

```
사람이 리더 암을 잡음  ──UDP 47800──>  follower_teleop_node ──> 서보
SmolVLA 정책          ──UDP 47800──>  follower_teleop_node ──> 서보
                          같은 규약 · 같은 포트 · 같은 안전장치
```

---

## ⚠️ 먼저 읽을 것 — 교시 자세가 무효입니다

2026-08-29 에 **LeRobot 캘리브레이션이 서보의 `Homing_Offset` 을 덮어썼습니다**
(`lerobot/motors/feetech/feetech.py:275`).

```
Present_Position = Actual_Position - Homing_Offset
```

`floor_grasp_profiles.py` 의 RAW 교시 자세가 같은 숫자로 **다른 물리 자세**를
가리킵니다. 되돌리려면:

```
python tools/arm/backup_servo_offsets.py COM8 --restore tools/arm/servo_backup/servo_COM8_20260829_181124.json
```

**둘 중 하나만 됩니다.** 되돌리면 베이스라인 미션이 살아나고 VLA 수집이
깨집니다. 그대로 두면 반대입니다. 어느 쪽으로 갈지 정하고 시작하세요.

리더는 `shoulder_pan` 가동폭이 2718 이고 팔로워는 2087 입니다. **텔레옵에서
좌우 끝까지 돌리면 팔로워가 한계에 부딪힙니다** — 시연 중에 그 자세를 쓰지
마세요.

### 이 브랜치에서는 arm_driver_node 를 기본값으로 못 띄웁니다

`arm_driver_node` 는 기동할 때 서보의 `Homing_Offset` 을
`floor_grasp_profiles.TAUGHT_HOMING_OFFSETS`(교시 당시 값)와 대조하고,
다르면 거부합니다. **이 브랜치는 VLA 캘리브레이션으로 도는 것이 정상**이라
그 검사에 걸립니다.

VLA 수집·추론에는 `arm_driver_node` 가 필요 없습니다 — 팔은
`follower_teleop_node` 가 잡습니다(둘은 같은 시리얼 포트를 못 나눠 쓰므로
어차피 동시에 못 뜹니다). bringup 은 `use_fake_arm:=true` 로 띄우세요.

굳이 띄워야 한다면 `-p verify_calibration:=false` 를 주되, **그 팔로는
베이스라인 교시 자세를 쓰지 마세요.** 지금 어느 캘리브레이션인지는 언제든
이렇게 봅니다.

```
python3 tools/arm/restore_taught_offsets.py
```

---

## 작업 규칙

파이 접속과 컨테이너 진입:

```
ssh pi@raspberrypi.local
cd ~/docker && ./exec_shell.sh
```

**컨테이너 셸은 zsh 입니다.** `setup.bash` 가 아니라 `setup.zsh` 를 source
해야 합니다. 그리고 어느 셸에서든 `ROS_DOMAIN_ID` 를 가장 먼저 export
합니다.

```
export ROS_DOMAIN_ID=21
source /opt/ros/humble/setup.zsh
source /ros2_ws/install/setup.zsh
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.zsh
```

배포·재시작 뒤에는 `perception_node` 가 떠 있는지 항상 확인합니다.

---

## 1단계 — 수집

### 그리퍼캠부터 띄웁니다

```
ros2 run grippers_perception gripper_cam_publisher_node
```

두 토픽이 나옵니다.

| 토픽 | 쓰는 곳 |
|---|---|
| `gripper_cam/image_raw` | 화면 확인용 (`live_yolo_demo.py`) |
| `gripper_cam/image_raw/compressed` | **녹화와 학습이 쓰는 것** |

압축본이 없으면 녹화가 성립하지 않습니다. bgr8 640×480 15Hz 원본은 **분당
약 0.8GB** 라 몇 분이면 디스크가 찹니다. JPEG 는 그 5% 안팎입니다.

> `perception_node` 의 `confirm_grasp` 서비스가 같은 `/dev/gripper_cam` 을
> 독점으로 엽니다. **둘은 동시에 못 뜹니다.** 미션은 그 서비스를 부르지
> 않으므로(저장소에 호출부가 없습니다) 이쪽을 켜는 것이 맞습니다.

### 텔레옵으로 시연합니다

노트북에서:

```
cd ~/grippers-teleop && ./teleop.sh --record
```

`f` 로 팔 추종을 켜고 시연하고, 끝나면 `f` 로 끕니다. **켜고 끄는 그 구간이
한 에피소드**입니다 — 꺼진 동안의 팔 자세는 학습에 안 들어갑니다.

### 찍자마자 확인합니다

```
python3 tools/vla/bag_to_lerobot.py /grippers/recordings/demo_20260830_141230 --dry-run
```

`--dry-run` 은 lerobot 없이 돌아서 파이 안에서 바로 됩니다. **시연을 다
찍고 나서 데이터가 못 쓰는 것이었다는 걸 알면 되돌릴 수 없습니다.**

```
기준 프레임(그리퍼캠) 1834개 · 122.3초
에피소드 4개 · 프레임 1702개
  #0  프레임 431개  [52:483]
  #1  프레임 402개  [510:912]
  ...
```

에피소드가 0개로 나오면 원인은 둘 중 하나입니다 — 추종을 안 켜고 찍었거나,
`--state-period 0` 으로 찍혀 `observation.state` 가 비었거나.

---

## 2단계 — 변환

```
python3 tools/vla/bag_to_lerobot.py <bag> --repo-id kica927/grippers-pick --task "체스말을 집어 바구니에 넣는다" --out ~/datasets/grippers-pick
```

무엇이 무엇이 되는지:

| 녹화 토픽 | LeRobot 필드 | |
|---|---|---|
| `gripper_cam/image_raw/compressed` | `observation.images.gripper` | 기준 시계 |
| `/teleop/follower_present` | `observation.state` | 팔이 **실제로** 있는 곳 |
| `/teleop/follower_counts` | `action` | 팔에 **내린 목표** |
| `/teleop/engaged` | 에피소드 경계 | |

**state 와 action 을 헷갈리면 안 됩니다.** state 자리에 명령을 넣으면 정책이
자기 출력을 관측으로 되먹는 것을 배웁니다. 그래서 실측 자세가 없는 녹화는
변환기가 거부합니다 — 조용히 명령으로 대체하지 않습니다.

판단 로직은 전부 [`episode_spec.py`](episode_spec.py) 에 있고 ROS 없이
테스트됩니다(`tests/test_vla_episode_spec.py`, 22개).

---

## 3단계 — 학습

파이에서 하지 않습니다. 맥이나 GPU 가 있는 기계에서 합니다.

```
pip install "lerobot==0.4.4"
lerobot-train --policy.path=lerobot/smolvla_base --dataset.repo_id=kica927/grippers-pick --batch_size=64 --steps=20000 --output_dir=outputs/smolvla-grippers
```

`lerobot/smolvla_base` 에서 파인튜닝합니다. 밑바닥부터 학습하지 않습니다 —
에피소드 수십 개로는 어림도 없습니다.

### ⚠️ 버전을 고정하는 이유

**이 코드는 lerobot 0.4.4 를 읽고 짰습니다.** 0.3.x 에서 **정규화가 정책
밖 전·후처리기로 옮겨 갔고**, 그 전후로 추론 호출 방식이 다릅니다. 버전을
내리면 정책 노드가 조용히 틀린 값을 내보냅니다 — 예외가 안 납니다.

학습한 기계와 파이의 lerobot 버전이 같아야 합니다.

---

## 4단계 — 추론

체크포인트를 파이로 옮기고, 컨테이너에서:

```
ros2 run grippers_vla smolvla_policy_node --ros-args -p policy_path:=/grippers/models/smolvla-grippers -p dry_run:=true
```

**`dry_run:=true` 가 기본값이고, 처음에는 반드시 이대로 띄웁니다.** 관측과
추론과 청크 재생을 전부 하되 `engaged=False` 로만 보냅니다 — 팔은 안 움직이고,
정책이 무엇을 내려 했는지는 로그로 보입니다.

같이 떠 있어야 하는 것:

```
python3 tools/teleop/follower_teleop_node.py --arm-port /dev/soarm
ros2 run grippers_perception gripper_cam_publisher_node
```

로그가 이렇게 나오면 정상입니다.

```
[smolvla_policy_node] 추론 2840ms · 청크 50스텝 (1.0초 분량)
```

괜찮아 보이면 `dry_run:=false` 로 다시 띄웁니다.

### 액션 공간 — 정책이 내는 숫자가 무엇인가

**원시 서보 카운트입니다.** 정규화는 LeRobot 이 데이터셋 통계로 하고,
후처리기가 되돌려 줍니다. 우리는 그 값을 반올림해 그대로 목표로 씁니다.

| | |
|---|---|
| 액션 차원 | **6** — 데이터셋 차원으로 언패드됩니다(내부 패딩 32) |
| 청크 길이 | **50스텝** (`chunk_size` 기본값) = 50Hz 에서 1.0초 분량 |
| 값의 범위 | 0..4095 를 **벗어날 수 있습니다** — 아래 참고 |

#### 0/4095 이음매

학습 데이터는 `unwrap_series` 로 **펴서** 만듭니다. 이음매를 넘는 에피소드
에서는 값이 0..4095 밖으로 나가고, 정책은 그 공간에서 출력합니다. 반면
추론 때 기준으로 심는 실측 자세는 서보가 읽어 준 원시값입니다.

**두 값을 그냥 빼면 안 됩니다.** 같은 물리 자세가 4100 과 5 로 표현될 수
있고, 뺄셈은 +4095 를 줍니다 — 슬루에 걸려 **팔이 반대 방향으로 영원히
기어갑니다.** `ChunkPlayer` 는 최단 회전(`wrap_delta`)으로 계산합니다.
한 틱 이동이 슬루 80카운트로 잘리므로 최단 회전이 언제나 진짜 움직임입니다
— 반 바퀴를 20ms 에 도는 관절은 없습니다.

#### 적재할 때 체크포인트를 확인합니다

틀린 체크포인트(다른 로봇, 파인튜닝 안 한 `smolvla_base`)를 줘도 추론은
그냥 돕니다. 그래서 적재 시점에 **액션 차원이 6인지**와 **카메라 이름이
`observation.images.gripper` 인지**를 봅니다 — 첫 추론에서 알면 그때는 이미
팔이 잡혀 있습니다.

### 속도가 세 개입니다 — 섞으면 안 됩니다

| | 기본값 | 무엇이 정하나 |
|---|---|---|
| **액션 속도** `action_hz` | 15Hz | **데이터셋을 만든 fps** |
| **송신 속도** `send_hz` | 50Hz | 팔로워 데드맨(0.4초)과 부드러움 |
| 추론 속도 | 측정값 | Pi CPU 성능 |

학습 데이터를 카메라(15Hz) 기준으로 만들었으면 **청크의 한 스텝은 1/15초짜리
움직임**입니다. 그걸 50Hz 로 풀면 궤적이 3.3배 빨리 돌아갑니다 — 정책이 배운
속도가 아니고, 슬루에 계속 걸려 목표를 못 따라갑니다.

그래서 패킷은 50Hz 로 보내되 **한 액션을 3틱씩** 보냅니다. 반복이 낭비가
아닌 이유: 슬루가 한 틱에 80카운트만 허용하므로, 같은 목표를 여러 틱 보내는
것이 그 목표로 **속도 제한을 걸어 다가가는 것**입니다.

`bag_to_lerobot.py --fps` 를 바꾸면 `action_hz` 도 같이 바꿔야 합니다.

#### 추론이 느려도 청크가 먼저 마르지 않습니다

50스텝 청크가 15Hz 에서 **3.3초 분량**입니다. Pi CPU 추론이 2~3초여도 그
안에 들어옵니다. (50Hz 로 풀었다면 1.0초라 매번 말랐을 겁니다.)

낡음 판정도 절대 시간이 아니라 **청크가 덮는 시간 + 여유 1초**입니다 —
절대값으로 두면 긴 청크를 중간에 끊거나 죽은 정책의 궤적을 오래 재생합니다.

#### 해제되면 반드시 다시 잡습니다

팔로워는 해제되면 `if not self.tracking: return` 에 갇힙니다. **같은 epoch
로 engaged=True 를 아무리 보내도 다시 안 잡습니다** — 새 epoch 만이 latch 를
다시 겁니다. 그래서 해제될 때마다 epoch 를 올리고 **지금** 읽은 자세로 기준을
다시 잡습니다. 추론이 몇 초 걸렸으므로 추론 직전의 자세는 낡았을 수 있습니다.

이게 없으면 한 번 해제된 뒤로 패킷은 계속 나가는데 팔은 영영 안 움직입니다.

### 안전장치

정책이 직접 서보를 열지 않습니다. 팔로워 수신기를 통과하므로 **실기로
다듬은 안전장치를 그대로 물려받습니다** — 관절 한계 클램프, 한 패킷당 슬루
80카운트, 델타 추종(켜는 순간 안 튐), 0.4초 데드맨, 신호가 끊겨도 토크 유지.

그 위에 `action_chunk.py` 가 세 가지를 더 겁니다.

| | |
|---|---|
| 데드맨을 굶기지 않는다 | 새 액션이 없어도 마지막 값을 계속 보냅니다 |
| 슬루를 보내는 쪽에서도 건다 | 팔로워에서 잘리면 명령과 실제가 조용히 갈라집니다 |
| 낡은 청크는 안 쓴다 | 2초 넘으면 해제 — 정책이 세상을 안 보고 팔을 움직이는 구간을 막습니다 |

**그리고 어느 것도 진짜 비상정지가 아닙니다. 차체 전원 스위치가 진짜
비상정지입니다.** 정책을 멈추려고 노드를 죽이지 마세요 — 마지막 명령이
latch 된 채 남습니다.

---

## 검증한 것과 못 한 것

**하드웨어 없이 검증했습니다** (`tests/test_vla_*.py`, 93개)

- 카운트가 0/4095 를 넘을 때 큰 움직임으로 오해하지 않는다
- 추종이 꺼진 구간과 latch 직후 튐이 학습에 안 들어간다
- 결측 프레임을 0 이나 직전 값으로 메우지 않는다
- 청크가 떨어져도 데드맨을 굶기지 않는다
- 슬루를 넘는 명령이 잘리고, 잘렸다는 사실이 보고된다
- 낡은 청크로 팔을 움직이지 않는다
- 정책 노드가 서보를 직접 열지 않는다
- 학습과 추론의 색 순서(RGB)가 같다
- 녹화 한 벌이 프레임이 되기까지의 배관 전체 — 세 토픽을 카메라 시계에
  맞추고, 관절별로 펴고, 추종 구간만 잘라내는 순서 (bag 리더를 갈아 끼워
  ROS 없이 돌립니다)
- 못 쓰는 녹화를 못 쓴다고 말한다 — 영상 없음 · 추종 안 켬 · 실측 자세 없음
- 0/4095 이음매를 넘는 목표를 최단 회전으로 본다 (양방향)
- 틀린 체크포인트를 적재 시점에 거부한다
- 액션 속도와 송신 속도를 섞지 않는다 — 한 액션을 3틱씩 보낸다
- 해제된 뒤 새 epoch 로 다시 잡는다

**소스를 읽어 확인했습니다** (lerobot 0.4.4 휠, 2026-08-30)

실행이 아니라 **소스 대조**입니다. 그래도 적어 두는 이유는, 여기서 틀리면
학습을 다 끝낸 뒤 실기 당일에야 드러나기 때문입니다.

| 확인한 것 | 어디서 |
|---|---|
| 정책과 전·후처리기를 같이 연다 | `async_inference/policy_server.py:154-166` |
| 후처리기는 스텝마다 `(B, action_dim)` | 같은 파일 `_predict_action_chunk` |
| `predict_action_chunk` → `(B, T, D)` | `policies/smolvla/modeling_smolvla.py:311` |
| `add_frame(frame)` — `task` 는 프레임 안의 키 | `datasets/lerobot_dataset.py:1171`, `datasets/utils.py:986` |
| 피처 이름 `["height","width","channels"]` | `datasets/utils.py:661` |
| 이미지는 [0,1] float, CHW, 배치차원 | `policies/utils.py:98` |
| 액션 차원은 데이터셋 차원으로 언패드 | `modeling_smolvla.py:295` |
| `chunk_size` 기본값 50 | `configuration_smolvla.py:32` |

`tests/test_vla_lerobot_contract.py` 가 이 호출들을 못 박습니다 — 누가
'정리'하면 그때 걸립니다.

**실기로 확인하지 못했습니다**

- `--state-period` 1/15초가 50Hz 텔레옵의 직렬 버스 예산 안에 드는지.
  `get_all_positions()` 은 sync read 가 아니라 서보 6개 순차 왕복입니다.
  **텔레옵이 버벅이면 이것부터 늘리거나 0 으로 꺼 보세요.**
- Pi 5 CPU 에서 SmolVLA 추론이 몇 초인지. 청크가 3.3초를 덮으므로 그
  안이면 끊기지 않습니다. **로그의 `추론 ...ms` 를 보고 3.3초를 넘으면**
  `action_hz` 를 낮추거나(궤적이 느려집니다) 청크를 길게 쓰세요.
- 정책이 실제로 무언가를 집는지. **데이터셋이 아직 없습니다.**

**측정하지 않은 숫자는 쓰지 않습니다.**
