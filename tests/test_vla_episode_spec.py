"""녹화를 학습 데이터로 바꿀 때 지켜야 하는 것 (2026-08-30, VLA 스트레치).

여기서 고정하는 성질은 셋이다 — **한 바퀴 도는 카운트를 움직임으로 오해하지
않는다 · 조작자가 손을 뗀 구간을 학습에 넣지 않는다 · 빈칸을 지어내지
않는다.** 셋 다 조용히 틀리는 종류라, 학습을 다 돌리고 나서야 정책이
이상하다는 것으로만 드러난다.
"""

import sys
from pathlib import Path

import pytest

_VLA = Path(__file__).resolve().parent.parent / "tools" / "vla"
sys.path.insert(0, str(_VLA))

import episode_spec as spec  # noqa: E402


# ── ① 카운트 한 바퀴 ───────────────────────────────────────────────────────


def test_한_바퀴_넘는_지점을_큰_움직임으로_보지_않는다():
    """4090 -> 5 는 +11 이지 -4085 가 아니다.

    편지 않고 넣으면 정책은 아무 일도 없던 순간에 팔이 반대편으로 날아간
    것을 배운다."""
    out = spec.unwrap_series([4090, 4095, 5, 10])
    steps = [b - a for a, b in zip(out, out[1:])]

    assert steps == [5, 6, 5]
    assert max(abs(s) for s in steps) < spec.POS_HALF


def test_첫_값은_원래_카운트를_유지한다():
    """절대 의미가 없어지면 에피소드마다 좌표계가 달라진다."""
    assert spec.unwrap_series([2048, 2050])[0] == 2048


def test_결측은_자리를_지키고_누적을_끊지_않는다():
    """결측을 건너뛰고 이어 붙이면 없던 움직임이 생긴다."""
    out = spec.unwrap_series([100, None, 120, spec.MISSING, 140])

    assert out[1] is None and out[3] is None
    assert [out[0], out[2], out[4]] == [100, 120, 140]


def test_평범한_수열은_그대로_둔다():
    assert spec.unwrap_series([1000, 1010, 1020]) == [1000, 1010, 1020]


# ── ② 추종이 켜져 있던 구간만 ──────────────────────────────────────────────


def _flags(pattern: str):
    """'..###..' 같은 그림으로 engaged 열을 만든다."""
    return [c == "#" for c in pattern]


def test_켜져_있던_구간만_에피소드가_된다():
    eps = spec.engaged_episodes(_flags("..####################.."),
                                settle_frames=0, min_frames=1)

    assert len(eps) == 1
    assert (eps[0].start, eps[0].end) == (2, 22)


def test_켠_직후_몇_프레임은_버린다():
    """latch 순간 기준점이 잡히면서 목표가 한 번 튄다 — 조작자의 의도가
    아니다."""
    eps = spec.engaged_episodes(_flags("#" * 20), settle_frames=3, min_frames=1)

    assert eps[0].start == 3


def test_너무_짧은_구간은_에피소드가_아니다():
    """잘못 눌렀다 뗀 것이다."""
    eps = spec.engaged_episodes(_flags("..###.."), settle_frames=0, min_frames=15)

    assert eps == []


def test_여러_번_켰다_끄면_에피소드도_여러_개다():
    eps = spec.engaged_episodes(_flags("#" * 20 + "." * 5 + "#" * 20),
                                settle_frames=0, min_frames=5)

    assert [e.index for e in eps] == [0, 1]
    assert eps[1].start == 25


def test_끝까지_켜진_채_끝나도_구간이_닫힌다():
    """녹화를 추종 중에 끊는 것이 오히려 정상이다."""
    eps = spec.engaged_episodes(_flags("#" * 30), settle_frames=0, min_frames=5)

    assert len(eps) == 1 and eps[0].end == 30


def test_켜졌다는_증거가_없으면_꺼진_것으로_본다():
    """engaged 토픽이 아직 한 번도 안 온 앞 구간이 여기 해당한다."""
    eps = spec.engaged_episodes([None] * 10 + [True] * 20,
                                settle_frames=0, min_frames=5)

    assert eps[0].start == 10


# ── ③ 서로 다른 주기를 한 시계로 ───────────────────────────────────────────


def test_직전_값을_유지한다():
    """명령은 다음 명령이 올 때까지 유효하다 — 서서히 변한 것이 아니다."""
    out = spec.hold_sample(ref_stamps=[0.0, 0.1, 0.2, 0.3],
                           stamps=[0.05, 0.25], values=["a", "b"])

    assert out == [None, "a", "a", "b"]


def test_보간하지_않는다():
    """중간값을 지어내면 정책이 실제로는 없었던 자세를 배운다."""
    out = spec.hold_sample([0.0, 0.5, 1.0], [0.0, 1.0], [0, 100])

    assert out == [0, 0, 100]


def test_아직_아무_값도_없으면_비워_둔다():
    """앞에서 끌어오면 녹화 시작 전의 자세를 지어내는 것이 된다."""
    assert spec.hold_sample([0.0, 1.0], [2.0], ["늦게"]) == [None, None]


def test_같은_시각이면_그_값을_쓴다():
    assert spec.hold_sample([1.0], [1.0], ["동시"]) == ["동시"]


# ── 프레임 조립 — 빈칸을 메우지 않는다 ─────────────────────────────────────


def _ok(v):
    return [v] * spec.JOINT_COUNT


def test_결측이_있는_프레임은_버린다():
    """0 이나 직전 값으로 메우면 정책이 없던 움직임을 배운다."""
    ep = spec.Episode(start=0, end=3)
    state = [_ok(1), [1, 1, None, 1, 1, 1], _ok(3)]
    action = [_ok(10), _ok(20), _ok(30)]

    rows, dropped_state, dropped_action = spec.build_frames(ep, state, action)

    assert [r.ref_index for r in rows] == [0, 2]
    assert dropped_state == 1 and dropped_action == 0


def test_state_와_action_을_섞지_않는다():
    """state 자리에 명령을 넣으면 정책이 자기 출력을 관측으로 되먹는다."""
    ep = spec.Episode(start=0, end=1)
    rows, _, _ = spec.build_frames(ep, [_ok(7)], [_ok(99)])

    assert rows[0].state == _ok(7)
    assert rows[0].action == _ok(99)


def test_state_가_통째로_없으면_전부_버린다():
    """--state-period 0 으로 찍은 녹화가 여기 해당한다. 조용히 명령으로
    대체하지 않는다."""
    ep = spec.Episode(start=0, end=5)
    rows, dropped, _ = spec.build_frames(ep, [None] * 5, [_ok(1)] * 5)

    assert rows == [] and dropped == 5


# ── LeRobot 규약 ───────────────────────────────────────────────────────────


def test_LeRobot_이_찾는_이름을_쓴다():
    """SmolVLA 는 이 이름으로 찾는다 — 다르면 조용히 못 찾는다."""
    f = spec.lerobot_features(["gripper", "top"])

    assert "observation.state" in f
    assert "action" in f
    assert "observation.images.gripper" in f
    assert "observation.images.top" in f


def test_관절_수가_상태와_액션에_같이_박힌다():
    f = spec.lerobot_features([])

    assert f["observation.state"]["shape"] == (spec.JOINT_COUNT,)
    assert f["action"]["shape"] == (spec.JOINT_COUNT,)


def test_이미지는_video_로_저장한다():
    """프레임을 낱장 png 로 두면 에피소드 수십 개에서 용량이 감당이 안 된다."""
    f = spec.lerobot_features(["gripper"])

    assert f["observation.images.gripper"]["dtype"] == "video"


# ── 사본 상수가 원본과 어긋나지 않는지 ─────────────────────────────────────


def test_관절_정의가_driver_sdk_와_같다():
    """episode_spec 은 맥에서도 돌아야 해서 driver_sdk 를 import 하지 않고
    관절 정의를 다시 적는다. 그 사본이 원본과 갈라지면 데이터셋의 관절
    순서가 팔과 달라진다 — 조용히 틀리고, 학습을 다 돌린 뒤에야 안다."""
    sdk = (Path(__file__).resolve().parent.parent
           / "third_party" / "soarm_provided_d" / "soarm_lab" / "driver_sdk.py")
    if not sdk.exists():
        pytest.skip("driver_sdk.py 없음 (서브모듈 미체크아웃)")

    ns: dict = {}
    for line in sdk.read_text(encoding="utf-8").splitlines():
        if line.startswith(("JOINT_NAMES =", "JOINT_IDS =")):
            exec(line, ns)  # noqa: S102 — 리터럴 두 줄만 평가한다

    assert ns["JOINT_IDS"] == spec.JOINT_IDS
    assert ns["JOINT_NAMES"] == spec.JOINT_NAMES


def test_wrap_delta_가_teleop_protocol_과_같다():
    """팔로워가 쓰는 식과 다르면 보낸 목표와 실행된 목표가 갈라진다."""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                          / "tools" / "teleop"))
    import teleop_protocol

    for a, b in [(4090, 5), (5, 4090), (0, 0), (2048, 0), (100, 200)]:
        assert spec.wrap_delta(a, b) == teleop_protocol.wrap_delta(a, b)
