"""정책 액션을 팔로 내보낼 때 지켜야 하는 것 (2026-08-30, VLA 스트레치).

여기서 고정하는 성질은 셋이다 — **데드맨을 굶기지 않는다 · 물리적으로
불가능한 명령을 조용히 흘리지 않는다 · 정책이 세상을 안 보는 동안 팔을
움직이지 않는다.**

셋 다 팔이 실제로 움직이는 경로라, 틀리면 로그가 아니라 물건이 부서진다.
"""

import sys
from pathlib import Path

_VLA = Path(__file__).resolve().parent.parent / "tools" / "vla"
sys.path.insert(0, str(_VLA))

import action_chunk as ac  # noqa: E402

HOME = [2048] * 6


def _player(**kw):
    """페이싱과 무관한 테스트용 — 한 틱에 한 액션(action_hz == send_hz).

    페이싱 자체는 아래 '속도 세 개' 절에서 따로 본다. 섞으면 한 테스트가
    두 가지를 보게 되고, 깨졌을 때 어느 쪽인지 알 수 없다."""
    kw.setdefault("action_hz", 50.0)
    kw.setdefault("send_hz", 50.0)
    p = ac.ChunkPlayer(**kw)
    p.prime(HOME)
    return p


def _chunk(n, step=10):
    """관절이 한 스텝에 step 카운트씩 같은 방향으로 가는 청크."""
    return [[2048 + step * (i + 1)] * 6 for i in range(n)]


# ── ① 데드맨을 굶기지 않는다 ───────────────────────────────────────────────


def test_청크가_떨어져도_계속_보낸다():
    """0.4초 끊기면 팔로워 데드맨이 걸려 추종이 해제된다. 보낼 새 액션이
    없는 것과 링크가 죽은 것은 다르다."""
    p = _player()
    p.submit(_chunk(2), now=0.0)
    for t in (0.02, 0.04):
        p.tick(t)

    tick = p.tick(0.06)

    assert tick.engaged is True
    assert tick.counts == [2068] * 6, "마지막 액션을 붙들고 있어야 한다"


def test_아직_아무_청크도_없으면_추종을_켜지_않는다():
    """첫 추론이 끝나기 전에 팔을 잡으면 안 된다."""
    p = ac.ChunkPlayer()

    tick = p.tick(0.0)

    assert tick.engaged is False
    assert tick.counts == []


def test_현재_자세를_심어_두면_첫_틱이_그_자리에서_출발한다():
    """심지 않으면 정책의 첫 액션이 슬루 없이 통째로 나간다 — 팔이 튄다."""
    p = _player()
    p.submit([[2500] * 6], now=0.0)

    tick = p.tick(0.0)

    assert tick.counts == [2048 + ac.SLEW_COUNTS_DEFAULT] * 6


# ── ② 슬루를 여기서도 건다 ────────────────────────────────────────────────


def test_한_틱에_슬루보다_많이_움직이지_않는다():
    """팔로워도 자르지만, 거기서 잘리면 명령과 실제가 조용히 갈라진다."""
    p = _player(slew=80)
    p.submit([[3000] * 6], now=0.0)

    tick = p.tick(0.0)

    assert tick.counts == [2128] * 6
    assert tick.clamped == 6, "잘랐다는 사실이 보고돼야 한다"


def test_반대_방향도_같은_한도로_자른다():
    p = _player(slew=80)
    p.submit([[1000] * 6], now=0.0)

    assert p.tick(0.0).counts == [1968] * 6


def test_한도_안의_명령은_그대로_나간다():
    p = _player(slew=80)
    p.submit([[2100] * 6], now=0.0)

    tick = p.tick(0.0)

    assert tick.counts == [2100] * 6
    assert tick.clamped == 0


def test_잘린_횟수가_누적된다():
    """정책이 계속 불가능한 값을 내고 있다는 것은 사람이 알아야 한다."""
    p = _player(slew=80)
    p.submit([[3000] * 6, [3000] * 6], now=0.0)
    p.tick(0.0)
    p.tick(0.02)

    assert p.clamped_total == 12


def test_기본_슬루가_팔로워와_같다():
    """여기가 더 크면 팔로워가 조용히 자른다 — 그 순간 이 파일의 계측이
    거짓말이 된다."""
    node = (Path(__file__).resolve().parent.parent
            / "tools" / "teleop" / "follower_teleop_node.py")
    text = node.read_text(encoding="utf-8")

    assert f'default={ac.SLEW_COUNTS_DEFAULT}' in text


# ── 0/4095 이음매 ──────────────────────────────────────────────────────────
#
# 학습 데이터는 unwrap 해서 만들므로 정책 출력이 0..4095 를 벗어날 수 있다.
# 반면 prime() 에 들어오는 실측 자세는 서보가 읽어 준 원시값이다. 두 공간을
# 그냥 빼면 같은 자세가 4095 카운트 차이로 보인다.


def test_이음매_너머의_목표를_최단_회전으로_본다():
    """정책이 4100(=4)을 냈고 팔은 5에 있다. 실제 이동은 -1 이지 +4095 가
    아니다. 뺄셈을 쓰면 슬루에 걸려 팔이 반대 방향으로 기어간다."""
    p = ac.ChunkPlayer(slew=80)
    p.prime([5] * 6)
    p.submit([[4100] * 6], now=0.0)

    tick = p.tick(0.0)

    assert tick.counts == [4] * 6
    assert tick.clamped == 0, "정상 이동을 잘랐다면 방향 계산이 틀린 것이다"


def test_아래로_이음매를_넘어간다():
    """4090 에서 10 으로 = +16."""
    p = ac.ChunkPlayer(slew=80)
    p.prime([4090] * 6)
    p.submit([[10] * 6], now=0.0)

    assert p.tick(0.0).counts == [10] * 6


def test_위로_이음매를_넘어간다():
    """10 에서 4090 으로 = -16."""
    p = ac.ChunkPlayer(slew=80)
    p.prime([10] * 6)
    p.submit([[4090] * 6], now=0.0)

    assert p.tick(0.0).counts == [4090] * 6


def test_보내는_값은_항상_0_4095_안이다():
    """팔로워도 다시 감지만, 우리 쪽 계측이 범위 밖 숫자를 들고 있으면
    로그를 읽는 사람이 무엇이 일어났는지 알 수 없다."""
    p = ac.ChunkPlayer(slew=1000)
    p.prime([4000] * 6)
    p.submit([[5000] * 6, [9000] * 6], now=0.0)

    for _ in range(2):
        assert all(0 <= v < ac.POS_RANGE for v in p.tick(0.0).counts)


def test_이음매를_넘어도_슬루는_그대로_걸린다():
    """최단 회전으로 봐도 여전히 먼 목표는 잘려야 한다."""
    p = ac.ChunkPlayer(slew=80)
    p.prime([0] * 6)
    p.submit([[1000] * 6], now=0.0)

    tick = p.tick(0.0)

    assert tick.counts == [80] * 6
    assert tick.clamped == 6


def test_최단_회전_정의가_텔레옵과_같다():
    """팔로워도 wrap_delta 로 리더 변화량을 잰다. 식이 다르면 우리가 보낸
    목표와 팔이 실제로 가는 곳이 갈라진다."""
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parent.parent
                            / "tools" / "teleop"))
    import teleop_protocol

    assert ac.POS_RANGE == teleop_protocol.POS_RANGE
    for a, b in [(4100 % 4096, 5), (10, 4090), (0, 0), (2048, 0)]:
        assert ac.wrap_delta(a, b) == teleop_protocol.wrap_delta(a, b)


# ── ③ 정책이 세상을 안 보는 동안 움직이지 않는다 ───────────────────────────


def test_낡은_청크는_재생하지_않는다():
    """추론이 멈췄는데 남은 청크를 끝까지 풀면, 정책이 관측 없이 팔을
    움직이는 구간이 생긴다."""
    p = _player(stale_grace_s=1.0)
    p.submit(_chunk(200), now=0.0)      # 200스텝 @50Hz = 4.0초 분량
    p.tick(0.02)

    tick = p.tick(5.5)                  # 4.0 + 여유 1.0 을 넘겼다

    assert tick.engaged is False
    assert "낡" in tick.reason


def test_해제해도_마지막_자세를_계속_보낸다():
    """정지가 아니라 해제다 — 팔로워는 토크를 유지한 채 그 자리에 선다.
    여기서 팔을 놓으면 들고 있던 물건이 떨어진다."""
    p = _player(stale_grace_s=0.5)
    p.submit(_chunk(200), now=0.0)
    p.tick(0.0)

    tick = p.tick(9.0)

    assert tick.counts == [2058] * 6


def test_새_청크가_한참_안_오면_붙들기를_그만둔다():
    p = _player(max_hold_s=1.0)
    p.submit(_chunk(1), now=0.0)
    p.tick(0.0)

    assert p.tick(0.5).engaged is True
    assert p.tick(1.5).engaged is False


def test_새_청크가_오면_남은_옛_청크를_버린다():
    """새 관측으로 낸 것이 언제나 더 옳다."""
    p = _player()
    p.submit(_chunk(50), now=0.0)
    p.tick(0.0)
    p.submit([[2048] * 6], now=0.1)

    assert p.remaining == 1
    assert p.tick(0.1).counts == [2048] * 6


def test_새_청크가_오면_나이도_다시_센다():
    p = _player(stale_grace_s=0.5)
    p.submit(_chunk(50), now=0.0)       # 50스텝 @50Hz = 1.0초 분량
    p.tick(0.0)
    p.submit(_chunk(50), now=3.0)       # 옛 청크였다면 이미 낡았을 시각

    assert p.tick(3.0).engaged is True


# ── 청크 소모 ──────────────────────────────────────────────────────────────


def test_청크를_순서대로_하나씩_쓴다():
    p = ac.ChunkPlayer(slew=1000, action_hz=50.0, send_hz=50.0)
    p.prime([0] * 6)
    p.submit([[10] * 6, [20] * 6, [30] * 6], now=0.0)

    assert [p.tick(0.0).counts[0] for _ in range(3)] == [10, 20, 30]
    assert p.remaining == 0


# ── 속도 세 개 — 액션·송신·추론 ────────────────────────────────────────────
#
# 액션 속도는 데이터셋 fps 다. 송신 속도는 데드맨이 정한다. 둘을 섞으면
# 정책이 배운 속도로 안 움직인다.


def test_데이터셋_fps_로_풀어낸다():
    """15fps 데이터셋의 한 스텝은 1/15초짜리 움직임이다. 50Hz 로 풀면
    궤적이 3.3배 빨리 돌아간다 — 정책이 배운 속도가 아니다."""
    p = ac.ChunkPlayer(action_hz=15.0, send_hz=50.0)

    assert p.ticks_per_action == 3


def test_한_액션을_여러_틱_보낸다():
    """슬루가 한 틱에 80카운트만 허용하므로, 같은 목표를 여러 틱 보내는
    것은 그 목표로 속도 제한을 걸어 다가가는 것이다."""
    p = ac.ChunkPlayer(slew=1000, action_hz=25.0, send_hz=50.0)
    p.prime([0] * 6)
    p.submit([[100] * 6, [200] * 6], now=0.0)

    seen = [p.tick(0.0).counts[0] for _ in range(4)]

    assert seen == [100, 100, 200, 200]
    assert p.remaining == 0


def test_청크가_덮는_시간이_액션_속도로_정해진다():
    """50스텝 청크는 15fps 에서 3.3초다 — Pi CPU 추론 2~3초가 그 안에
    들어온다. 50Hz 로 풀었다면 1.0초라 매번 말랐을 것이다."""
    p = ac.ChunkPlayer(action_hz=15.0, send_hz=50.0)
    p.prime(HOME)
    p.submit(_chunk(50), now=0.0)

    assert 3.3 < p.chunk_duration_s() < 3.4


def test_낡음_판정이_청크_길이에_따라간다():
    """절대 시간으로 두면 긴 청크를 중간에 끊거나 죽은 정책의 궤적을 오래
    재생한다."""
    p = ac.ChunkPlayer(action_hz=15.0, send_hz=50.0, stale_grace_s=1.0)
    p.prime(HOME)
    p.submit(_chunk(50), now=0.0)       # 3.33초 분량

    p.tick(0.0)

    assert p.tick(4.0).engaged is True, "아직 덮는 시간 안이다"
    assert p.tick(5.0).engaged is False, "3.33 + 1.0 을 넘겼다"


def test_액션_속도가_송신보다_빠르면_건너뛰지_않는다():
    """액션을 버리면 정책이 낸 궤적의 일부를 잃는다 — 속도를 맞추는 것과
    다른 일이다. 최소 1틱은 보낸다."""
    p = ac.ChunkPlayer(action_hz=100.0, send_hz=50.0)

    assert p.ticks_per_action == 1


def test_기본_액션_속도가_데이터셋_fps_와_같다():
    """둘이 갈라지면 정책이 배운 속도로 안 움직이는데, 증상이 '좀 이상하다'
    뿐이라 원인을 찾기 어렵다."""
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parent.parent
                            / "tools" / "vla"))
    import bag_to_lerobot

    assert ac.ACTION_HZ_DEFAULT == float(bag_to_lerobot.FPS_DEFAULT)
