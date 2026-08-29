# -*- coding: utf-8 -*-
"""녹화된 텔레옵을 SmolVLA 학습 데이터로 바꾸는 **순수 로직**.

ROS 도 torch 도 lerobot 도 import 하지 않는다. 그래서 맥에서, 하드웨어
없이, 컨테이너 밖에서 테스트가 돈다 — 이 저장소가 도메인 계층을 그렇게
분리해 둔 것과 같은 이유다.

bag 을 읽고 데이터셋을 쓰는 쪽은 `bag_to_lerobot.py` 이고, 여기서는
**무엇이 한 프레임이 되는가**만 정한다.

## 왜 이 세 가지가 필요한가

녹화된 토픽은 세 가지 이유로 그대로 쓸 수 없다.

**① 카운트가 0/4095 에서 한 바퀴 돈다.** 4090 -> 5 는 물리적으로 +11 인데
숫자로는 -4085 다. 정책이 이것을 보면 아무것도 안 한 순간에 팔이 최대
속도로 반대편으로 날아간 것을 배운다. `unwrap_series` 가 이것을 편다.

**② 조작자가 손을 뗀 구간이 섞여 있다.** 텔레옵은 `f` 로 추종을 켜고
끄는데, 꺼진 동안의 팔 자세는 "정책이 내렸어야 할 명령"이 아니라 그냥
멈춰 있는 팔이다. 학습에 넣으면 정지를 정답으로 배운다.
`engaged_episodes` 가 켜져 있던 구간만 잘라낸다.

**③ 토픽마다 주기가 다르다.** 카메라 15Hz, 텔레옵 50Hz, 라이다 10Hz 다.
LeRobot 데이터셋은 프레임마다 모든 필드가 있어야 하므로 하나의 시계에
맞춰야 한다. 카메라를 기준 시계로 삼고 나머지는 **직전 값 유지**로
채운다(`hold_sample`) — 명령 신호는 다음 명령이 올 때까지 유효하다는
뜻이므로 선형 보간이 아니라 계단 보간이 맞다.

## 무엇을 state 로, 무엇을 action 으로 쓰는가

    observation.state   /teleop/follower_present  팔이 지금 실제로 있는 곳
    action              /teleop/follower_counts   팔에 내린 목표

둘을 헷갈리면 정책이 자기 출력을 관측으로 되먹는 것을 배운다. 실측 자세를
못 읽은 녹화(구버전, 또는 --state-period 0)는 state 가 비므로 이 모듈이
거부한다 — 조용히 명령으로 대체하지 않는다.

## 정규화는 하지 않는다

원시 카운트 그대로 넣는다. LeRobot 이 데이터셋 통계(평균/표준편차)로
정규화를 직접 하므로, 여기서 미리 손대면 통계가 이중으로 걸린다. 다만
①의 unwrap 은 정규화가 아니라 **신호를 물리적으로 옳게 만드는 것**이라
반드시 먼저 해야 한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# 서보 카운트는 0..4095 에서 한 바퀴 돈다.
POS_RANGE = 4096
POS_HALF = POS_RANGE // 2

# driver_sdk.JOINT_IDS / JOINT_NAMES 와 같은 순서다. 여기서 다시 적는 이유는
# driver_sdk 가 /third_party/soarm_provided_d 에 있어 파이 컨테이너 밖에서는
# import 되지 않기 때문이다 — 이 모듈은 맥에서도 돌아야 한다.
# 어긋나면 tests/test_vla_episode_spec.py 가 잡는다.
JOINT_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
JOINT_COUNT = len(JOINT_IDS)

# 읽기 실패를 -1 로 표시한다(teleop_ros_bridge.publish_arm 참고).
# 카운트는 0..4095 라 충돌하지 않는다.
MISSING = -1

# 추종을 켠 직후 몇 프레임은 버린다. latch 순간 기준점이 잡히면서 목표가
# 한 번 크게 튈 수 있고, 그 프레임은 조작자의 의도가 아니다.
SETTLE_FRAMES_DEFAULT = 3

# 이보다 짧은 구간은 에피소드로 치지 않는다. 15Hz 에서 15프레임 = 1초 —
# 그보다 짧으면 조작자가 잘못 눌렀다 뗀 것이다.
MIN_EPISODE_FRAMES_DEFAULT = 15


def wrap_delta(a: int, b: int) -> int:
    """a - b 를 -2048..2047 범위의 최단 회전으로. teleop_protocol 과 같은 식."""
    return (a - b + POS_HALF) % POS_RANGE - POS_HALF


def unwrap_series(counts: list) -> list:
    """0/4095 를 넘나드는 카운트 수열을 연속 신호로 편다.

    첫 값은 그대로 두고, 그 뒤로는 최단 회전 델타를 누적한다. 실제 관절이
    한 틱에 반 바퀴를 돌 수는 없으므로 최단 회전이 곧 진짜 움직임이다.

    결과는 0..4095 를 벗어날 수 있다 — 의도한 것이다. 팔로워는 목표를
    `wrap_position()` 으로 다시 감으므로 되돌려 보내도 안전하다.

    결측(None 또는 MISSING)은 그 자리에 None 을 남기고 누적을 이어간다 —
    결측 구간을 건너뛰며 값을 이어 붙이면 없는 움직임이 생긴다."""
    out: list = []
    prev_raw = None
    acc = None
    for c in counts:
        if c is None or c == MISSING:
            out.append(None)
            continue
        c = int(c)
        if prev_raw is None:
            acc = c
        else:
            acc += wrap_delta(c, prev_raw)
        prev_raw = c
        out.append(acc)
    return out


def hold_sample(ref_stamps: list, stamps: list, values: list):
    """기준 시계에 맞춰 **직전 값 유지**로 다시 샘플링한다.

    각 기준 시각마다 그 시각 **이하**의 마지막 값을 쓴다. 아직 아무 값도
    안 온 시각에는 None 이 들어간다 — 없는 값을 앞에서 끌어오지 않는다.

    선형 보간을 쓰지 않는 이유: 이 신호들은 명령과 이산 상태(engaged)라
    두 값 사이에 '중간'이 없다. 명령은 다음 명령이 올 때까지 그대로
    유효한 것이지, 서서히 변한 것이 아니다.

    두 시각열 모두 오름차순이라고 본다(rosbag2 가 그렇게 준다)."""
    out: list = []
    i = 0
    last = None
    for t in ref_stamps:
        while i < len(stamps) and stamps[i] <= t:
            last = values[i]
            i += 1
        out.append(last)
    return out


@dataclass
class Episode:
    """추종이 켜져 있던 한 구간. 기준 시계(카메라) 인덱스로 표현한다."""

    start: int          # 포함
    end: int            # 미포함
    index: int = 0      # 몇 번째 에피소드인가

    @property
    def frames(self) -> int:
        return self.end - self.start


def engaged_episodes(engaged: list,
                     settle_frames: int = SETTLE_FRAMES_DEFAULT,
                     min_frames: int = MIN_EPISODE_FRAMES_DEFAULT) -> list:
    """engaged 가 True 로 이어지는 구간을 에피소드로 자른다.

    앞의 `settle_frames` 는 버린다(latch 직후의 튐). 그러고도 `min_frames`
    보다 짧으면 에피소드로 치지 않는다.

    아직 아무 engaged 값도 안 온 구간(None)은 False 로 본다 — 켜졌다는
    증거가 없으면 꺼진 것이다."""
    spans: list = []
    run_start = None
    for i, flag in enumerate(list(engaged) + [None]):
        on = bool(flag) if flag is not None else False
        if on and run_start is None:
            run_start = i
        elif not on and run_start is not None:
            spans.append((run_start, i))
            run_start = None

    out: list = []
    for start, end in spans:
        start += settle_frames
        if end - start >= min_frames:
            out.append(Episode(start=start, end=end, index=len(out)))
    return out


@dataclass
class FrameRow:
    """LeRobot 한 프레임. 이미지는 여기 담지 않는다 — 인덱스만 갖고 있다가
    쓰는 쪽에서 원본 프레임을 꺼낸다(메모리)."""

    ref_index: int
    state: list
    action: list


@dataclass
class BuildReport:
    """무엇을 왜 버렸는지. 조용히 사라지는 프레임이 없어야 한다."""

    kept: int = 0
    dropped_missing_state: int = 0
    dropped_missing_action: int = 0
    episodes: list = field(default_factory=list)

    def summary(self) -> str:
        lines = [f"에피소드 {len(self.episodes)}개 · 프레임 {self.kept}개"]
        for ep in self.episodes:
            lines.append(f"  #{ep.index}  프레임 {ep.frames}개  "
                         f"[{ep.start}:{ep.end}]")
        dropped = self.dropped_missing_state + self.dropped_missing_action
        if dropped:
            lines.append(f"버린 프레임 {dropped}개 — "
                         f"state 결측 {self.dropped_missing_state} · "
                         f"action 결측 {self.dropped_missing_action}")
        return "\n".join(lines)


def build_frames(episode: Episode, state: list, action: list) -> tuple:
    """한 에피소드의 프레임 목록과 버린 내역.

    `state` 와 `action` 은 이미 기준 시계에 맞춰지고 unwrap 된, 프레임마다
    관절 6개짜리 리스트다. 어느 한 관절이라도 결측이면 그 프레임을 버린다 —
    빈칸을 0 이나 직전 값으로 메우면 정책이 있지도 않은 움직임을 배운다."""
    rows: list = []
    dropped_state = 0
    dropped_action = 0
    for i in range(episode.start, episode.end):
        s = state[i] if i < len(state) else None
        a = action[i] if i < len(action) else None
        if s is None or any(v is None for v in s):
            dropped_state += 1
            continue
        if a is None or any(v is None for v in a):
            dropped_action += 1
            continue
        rows.append(FrameRow(ref_index=i, state=list(s), action=list(a)))
    return rows, dropped_state, dropped_action


def lerobot_features(image_keys: list, image_shape: tuple = (480, 640, 3)) -> dict:
    """LeRobotDataset.create 에 넘길 features.

    이름은 LeRobot 규약을 그대로 따른다 — `observation.images.<이름>`,
    `observation.state`, `action`. SmolVLA 는 이 이름으로 찾는다."""
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (JOINT_COUNT,),
            "names": list(JOINT_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (JOINT_COUNT,),
            "names": list(JOINT_NAMES),
        },
    }
    for key in image_keys:
        features[f"observation.images.{key}"] = {
            "dtype": "video",
            "shape": tuple(image_shape),
            "names": ["height", "width", "channel"],
        }
    return features
