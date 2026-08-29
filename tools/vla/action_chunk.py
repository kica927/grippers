# -*- coding: utf-8 -*-
"""정책이 낸 액션 청크를 텔레옵 패킷 속도로 풀어내는 **순수 로직**.

torch 도 lerobot 도 import 하지 않는다. 실행기(`smolvla_leader.py`)가 추론
결과를 여기에 넣고, 여기서 나온 값을 그대로 UDP 로 보낸다.

## 왜 청크를 풀어야 하는가

SmolVLA 는 한 번 추론할 때 액션을 **한 개가 아니라 묶음으로** 낸다(기본
50스텝). 반대로 팔로워(`follower_teleop_node`)는 50Hz 로 패킷이 계속
와야 한다 — 0.4초 끊기면 데드맨이 걸려 추종이 해제된다.

두 속도가 다르다. 맥에서 SmolVLA 한 번 추론이 수백 ms 걸리는데 그동안
패킷을 안 보내면 데드맨이 걸린다. 그래서 **추론은 가끔, 송신은 항상**
이어야 하고, 그 사이를 메우는 것이 이 파일이다.

## 속도가 세 개다 — 섞으면 안 된다

    액션 속도  action_hz   청크의 한 스텝이 원래 몇 초 간격이었나
    송신 속도  send_hz     패킷을 몇 Hz 로 보내나
    추론 속도  (측정값)    한 번 추론에 몇 초 걸리나

**액션 속도는 데이터셋의 fps 다.** 학습 데이터를 카메라(15Hz) 기준으로
만들었으면 청크의 한 스텝은 1/15초짜리 움직임이다. 그걸 50Hz 로 풀면
궤적이 3.3배 빨리 돌아간다 — 정책이 배운 속도가 아니고, 슬루에 계속 걸려
목표를 못 따라간다.

**송신 속도는 데드맨과 부드러움이 정한다.** 50Hz 를 유지하되, 한 액션을
`send_hz / action_hz` 번 반복해서 보낸다. 반복이 낭비가 아닌 이유: 슬루가
한 틱에 80카운트만 허용하므로, 같은 목표를 여러 틱 보내는 것은 그 목표로
**속도 제한을 걸어 다가가는 것**이 된다.

이 둘을 맞추면 ③의 문제도 대부분 사라진다. 50스텝 청크가 15Hz 에서
3.3초 분량이라, Pi CPU 추론이 2~3초여도 청크가 먼저 마르지 않는다.

## 세 가지를 여기서 지킨다

**① 데드맨을 굶기지 않는다.** 보낼 새 액션이 없어도 마지막 값을 계속
낸다. 팔은 그 자리에 서 있고 링크는 살아 있다.

**② 슬루를 여기서도 건다.** 팔로워가 이미 한 패킷당 80카운트로 자르지만
(`follower_teleop_node.MAX_SLEW`), 거기서 잘리면 **명령과 실제가 조용히
달라진다.** 정책이 낸 값이 물리적으로 불가능하다는 사실은 보내는 쪽이
알아야 한다 — 여기서 자르고 세어 둔다.

**③ 오래된 청크는 안 쓴다.** 추론이 멈췄는데(모델이 죽었거나 링크가
끊겼거나) 남은 청크를 끝까지 재생하면, 정책이 세상을 안 보고 팔을
움직이는 구간이 생긴다. `max_chunk_age_s` 를 넘기면 추종을 **해제**한다 —
정지가 아니라 해제다. 팔로워는 토크를 유지한 채 그 자리에 선다(들고 있던
물건을 떨어뜨리지 않는다).

## 0/4095 이음매를 넘어서 계산한다

학습 데이터는 `episode_spec.unwrap_series` 로 **펴서** 만든다. 이음매를
넘는 에피소드에서는 그 값이 0..4095 를 벗어나고, 정책은 그 공간에서
출력한다. 반면 `prime()` 에 들어오는 실측 자세는 서보가 읽어 준 원시
0..4095 다.

그래서 두 값을 그냥 빼면 안 된다. 같은 물리 자세인데 4100 과 5 로 표현될
수 있고, 뺄셈은 +4095 를 준다 — 슬루에 걸려 팔이 **반대 방향으로 영원히
기어간다.** 최단 회전(`wrap_delta`)으로 계산하면 +9 다.

한 틱의 실제 이동은 슬루(80카운트)로 잘리므로 최단 회전이 언제나 진짜
움직임이다 — 반 바퀴(2048)를 20ms 에 도는 관절은 없다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from episode_spec import POS_RANGE, wrap_delta

# 팔로워의 --slew 기본값과 같다. 50Hz 에서 80카운트 ≈ 350°/s.
# 여기 값이 팔로워보다 크면 팔로워가 조용히 자른다 — 같게 두는 것이 요점이다.
SLEW_COUNTS_DEFAULT = 80

# 액션 속도의 기본값. **데이터셋을 만든 fps 와 같아야 한다**
# (bag_to_lerobot.FPS_DEFAULT). 다르면 정책이 배운 속도로 안 움직인다.
ACTION_HZ_DEFAULT = 15.0
SEND_HZ_DEFAULT = 50.0

# 청크가 원래 덮는 시간을 이만큼 넘겨서까지 재생하지는 않는다. 절대 시간이
# 아니라 여유분인 이유: 청크 길이와 액션 속도에 따라 덮는 시간이 달라지는데,
# 절대값으로 두면 멀쩡한 청크를 중간에 끊거나(짧게 잡으면) 죽은 정책의
# 궤적을 오래 재생한다(길게 잡으면).
STALE_GRACE_S_DEFAULT = 1.0

# 청크가 소진된 뒤 마지막 액션을 붙들고 있는 한도. 이 시간까지는 팔이 그
# 자세로 버티고, 넘기면 해제한다.
MAX_HOLD_S_DEFAULT = 1.0


@dataclass
class Tick:
    """한 송신 시점에 보낼 것."""

    counts: list          # 관절 6개 목표 (해제 중이면 마지막 값)
    engaged: bool         # 팔 추종을 켠 채로 둘 것인가
    reason: str = ""      # 해제했다면 왜
    clamped: int = 0      # 이번 틱에 슬루로 잘린 관절 수


@dataclass
class ChunkPlayer:
    """액션 청크를 틱 단위로 풀어낸다.

    `submit()` 으로 새 청크를 넣고 `tick()` 을 송신 주기마다 부른다.
    시각은 밖에서 준다 — 테스트가 시계를 쥐고 있어야 하기 때문이다."""

    slew: int = SLEW_COUNTS_DEFAULT
    action_hz: float = ACTION_HZ_DEFAULT
    send_hz: float = SEND_HZ_DEFAULT
    stale_grace_s: float = STALE_GRACE_S_DEFAULT
    max_hold_s: float = MAX_HOLD_S_DEFAULT

    _chunk: list = field(default_factory=list)
    _cursor: int = 0
    _repeat: int = 0            # 지금 스텝을 몇 번 더 보내야 하나
    _chunk_at: float = 0.0
    _last: list = field(default_factory=list)   # 마지막으로 보낸 목표
    # 청크를 다 쓴 시각. **0.0 이 아니라 None 이 '아직 아님'이다** — 0.0 을
    # 센티널로 쓰면 t=0.0 에 소진된 경우가 falsy 로 걸려 붙들기 한도가 영영
    # 발동하지 않는다(tests/test_vla_action_chunk.py 가 잡았다).
    _exhausted_at: float | None = None
    clamped_total: int = 0

    @property
    def ticks_per_action(self) -> int:
        """한 액션을 몇 틱 동안 보낼 것인가 = send_hz / action_hz.

        1 미만으로 내려가지 않는다 — 액션을 건너뛰면 정책이 낸 궤적의
        일부를 버리는 것이고, 그건 속도를 맞추는 것과 다른 일이다."""
        if self.action_hz <= 0:
            return 1
        return max(1, int(round(self.send_hz / self.action_hz)))

    def chunk_duration_s(self, steps: int | None = None) -> float:
        """청크가 원래 덮는 시간."""
        n = len(self._chunk) if steps is None else steps
        return n / self.action_hz if self.action_hz > 0 else 0.0

    def submit(self, chunk, now: float) -> None:
        """새 청크로 갈아 끼운다. 남은 옛 청크는 버린다 — 새 관측으로 낸
        것이 언제나 더 옳다."""
        self._chunk = [list(a) for a in chunk]
        self._cursor = 0
        self._repeat = 0
        self._chunk_at = now
        self._exhausted_at = None

    def prime(self, counts) -> None:
        """추종을 켜기 전에 현재 팔 자세를 기준으로 심는다.

        이것을 안 하면 첫 틱에서 슬루가 '마지막 값 없음' 상태로 걸려,
        정책의 첫 액션이 통째로 나가 버린다."""
        self._last = [int(c) % POS_RANGE for c in counts]

    @property
    def remaining(self) -> int:
        return max(0, len(self._chunk) - self._cursor)

    def tick(self, now: float) -> Tick:
        age = now - self._chunk_at if self._chunk else float("inf")

        # ③ 오래된 청크는 안 쓴다. 기준은 절대 시간이 아니라 **청크가 원래
        #    덮는 시간 + 여유**다 — 그래야 청크가 길어져도 중간에 안 끊긴다.
        if self._chunk and age > self.chunk_duration_s() + self.stale_grace_s:
            return Tick(list(self._last), False,
                        f"청크가 {age:.1f}초 낡음 — 추론이 따라오지 못한다")

        if self.remaining > 0:
            want = self._chunk[self._cursor]
            # 같은 액션을 ticks_per_action 번 보낸다. 낭비가 아니다 —
            # 슬루가 한 틱에 80카운트만 허용하므로, 같은 목표를 여러 틱
            # 보내는 것이 그 목표로 속도 제한을 걸어 다가가는 것이다.
            self._repeat += 1
            if self._repeat >= self.ticks_per_action:
                self._cursor += 1
                self._repeat = 0
            if self.remaining == 0:
                self._exhausted_at = now
            return self._emit(want)

        # ① 청크가 없거나 다 썼다 — 마지막 값을 계속 낸다.
        if not self._last:
            return Tick([], False, "아직 보낼 액션이 없다")
        if (self._exhausted_at is not None
                and now - self._exhausted_at > self.max_hold_s):
            return Tick(list(self._last), False,
                        f"새 청크 없이 {now - self._exhausted_at:.1f}초 대기")
        return Tick(list(self._last), True)

    def _emit(self, want: list) -> Tick:
        """② 슬루를 걸고 보낸다. 잘린 관절 수를 같이 돌려준다."""
        if not self._last:
            self._last = [int(v) % POS_RANGE for v in want]
            return Tick(list(self._last), True)

        out = []
        clamped = 0
        for prev, target in zip(self._last, want):
            # 최단 회전으로 본다 — 위 "0/4095 이음매" 참고. 뺄셈을 쓰면
            # 이음매를 넘은 에피소드에서 팔이 반대로 기어간다.
            step = wrap_delta(int(target) % POS_RANGE, int(prev) % POS_RANGE)
            if step > self.slew:
                step, clamped = self.slew, clamped + 1
            elif step < -self.slew:
                step, clamped = -self.slew, clamped + 1
            out.append((int(prev) + step) % POS_RANGE)
        self._last = out
        self.clamped_total += clamped
        return Tick(list(out), True, clamped=clamped)
