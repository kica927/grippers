"""녹화 한 벌이 데이터셋 프레임이 되기까지 (2026-08-30, VLA 스트레치).

`episode_spec` 의 순수 함수들은 따로 검증돼 있다. 여기서 보는 것은 그것들을
**엮은 자리**다 — 세 토픽을 카메라 시계에 맞추고, 관절별로 펴고, 추종 구간만
잘라내는 순서. 실기에서 깨지는 것은 대개 엮은 자리다.

ROS 없이 돌리려고 `analyse(reader=...)` 로 bag 리더를 갈아 끼운다.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

_VLA = Path(__file__).resolve().parent.parent / "tools" / "vla"
sys.path.insert(0, str(_VLA))

import bag_to_lerobot as conv  # noqa: E402
import episode_spec as spec  # noqa: E402

HZ = 15.0


def _arr(values):
    return SimpleNamespace(data=list(values))


def _flag(value):
    return SimpleNamespace(data=bool(value))


def _jpeg(i):
    return SimpleNamespace(data=b"jpeg-%d" % i)


def _bag(*, frames=60, engaged_from=5, engaged_to=None, state=True,
         drift=10, top=None):
    """카메라 15Hz · 텔레옵 50Hz 로 찍힌 한 판을 흉내낸다."""
    engaged_to = frames if engaged_to is None else engaged_to
    cam_t = [i / HZ for i in range(frames)]
    tel_t = [i / 50.0 for i in range(int(frames / HZ * 50))]

    def counts(t, base):
        return [(base + int(t * drift)) % spec.POS_RANGE] * spec.JOINT_COUNT

    data = {
        conv.GRIPPER_TOPIC: {"t": cam_t, "v": [_jpeg(i) for i in range(frames)]},
        conv.ACTION_TOPIC: {"t": tel_t, "v": [_arr(counts(t, 2000)) for t in tel_t]},
        conv.ENGAGED_TOPIC: {
            "t": [cam_t[engaged_from], cam_t[engaged_to - 1]],
            "v": [_flag(True), _flag(False)],
        },
    }
    data[conv.STATE_TOPIC] = (
        {"t": tel_t, "v": [_arr(counts(t, 1990)) for t in tel_t]} if state
        else {"t": [], "v": []})
    if top is not None:
        data[top] = {"t": cam_t, "v": [_jpeg(i) for i in range(frames)]}

    def reader(bag, topics, storage_id):
        missing = [t for t in topics if t not in data]
        if missing:
            raise SystemExit(f"bag 에 없는 토픽: {missing}")
        return {t: data[t] for t in topics}

    return reader


# ── 배관이 실제로 프레임을 만든다 ──────────────────────────────────────────


def test_한_판이_에피소드_하나가_된다():
    frames, report, _data, ref_t = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, engaged_from=10))

    assert len(ref_t) == 90
    assert len(report.episodes) == 1
    assert report.kept > 0
    assert len(frames) == 1


def test_추종_전_구간은_들어가지_않는다():
    """조작자가 손을 대기 전의 팔은 정책이 내렸어야 할 명령이 아니다."""
    _frames, report, _data, _ref = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, engaged_from=30))
    ep = report.episodes[0]

    assert ep.start >= 30, "추종을 켠 뒤부터여야 한다"
    assert ep.start >= 30 + spec.SETTLE_FRAMES_DEFAULT, "latch 직후 튐도 버린다"


def test_state_와_action_이_서로_다른_토픽에서_온다():
    """섞이면 정책이 자기 출력을 관측으로 되먹는 것을 배운다."""
    frames, _report, _data, _ref = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90))
    row = frames[0][1][0]

    assert row.state != row.action
    assert row.state[0] == row.action[0] - 10, "합성 데이터의 10카운트 차이"


def test_50Hz_명령이_15Hz_프레임_수로_줄어든다():
    """카메라가 기준 시계다 — 이미지는 만들어낼 수 없고 명령은 늘릴 수 있다."""
    frames, report, _data, ref_t = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, engaged_from=0))

    assert len(ref_t) == 90
    assert report.kept <= 90
    assert all(r.ref_index < 90 for r in frames[0][1])


def test_카운트가_한_바퀴_돌아도_프레임이_이어진다():
    """4095 를 넘는 구간에서 정책이 큰 점프를 배우면 안 된다."""
    frames, _report, _data, _ref = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=120, engaged_from=0, drift=700))
    series = [r.action[0] for r in frames[0][1]]
    steps = [b - a for a, b in zip(series, series[1:])]

    assert steps, "프레임이 나와야 한다"
    assert max(abs(s) for s in steps) < spec.POS_HALF


# ── 못 쓰는 녹화는 못 쓴다고 말한다 ────────────────────────────────────────


def test_실측_자세가_없으면_프레임이_안_나온다():
    """--state-period 0 으로 찍힌 녹화. 조용히 명령으로 대체하지 않는다."""
    frames, report, _data, _ref = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, state=False))

    assert frames == []
    assert report.kept == 0
    assert report.dropped_missing_state > 0


def test_한_번도_안_켰으면_에피소드가_없다():
    reader = _bag(frames=90)
    frames, report, _d, _r = conv.analyse(
        Path("가짜"), None,
        reader=lambda b, t, s: {**reader(b, t, s),
                                conv.ENGAGED_TOPIC: {"t": [], "v": []}})

    assert report.episodes == [] and frames == []


def test_짧게_눌렀다_뗀_것은_에피소드가_아니다():
    _frames, report, _d, _r = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, engaged_from=10, engaged_to=15))

    assert report.episodes == []


def test_영상이_없으면_바로_멈춘다():
    """시연을 다 찍고 나서 알면 되돌릴 수 없다."""
    reader = _bag(frames=30)

    with pytest.raises(SystemExit) as exc:
        conv.analyse(Path("가짜"), None,
                     reader=lambda b, t, s: {**reader(b, t, s),
                                             conv.GRIPPER_TOPIC: {"t": [], "v": []}})

    assert "영상이 없" in str(exc.value)


def test_없는_토픽은_이름을_대고_멈춘다():
    with pytest.raises(SystemExit):
        conv.analyse(Path("가짜"), "/없는/토픽", reader=_bag(frames=30))


# ── 보고서 ─────────────────────────────────────────────────────────────────


def test_버린_프레임을_보고한다():
    """조용히 사라지는 프레임이 없어야 한다."""
    _frames, report, _d, _r = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, state=False))
    text = report.summary()

    assert "버린 프레임" in text
    assert "state 결측" in text


def test_요약이_에피소드_구간을_보여준다():
    _frames, report, _d, _r = conv.analyse(
        Path("가짜"), None, reader=_bag(frames=90, engaged_from=10))
    text = report.summary()

    assert "에피소드 1개" in text
    assert "프레임" in text


# ── 탑뷰를 붙였을 때 ───────────────────────────────────────────────────────


def test_두_번째_카메라도_같은_시계로_붙는다():
    top = "/depth_cam/image_raw/compressed"
    frames, report, data, _ref = conv.analyse(
        Path("가짜"), top, reader=_bag(frames=90, top=top))

    assert report.kept > 0
    assert top in data
    assert len(data[top]["v"]) == 90
