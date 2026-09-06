"""correlate_arm_motor_stall.py — 그리퍼/팔 버스 정지와 바퀴 모터 워치독의
인과관계 확증 도구 (2026-09-06, 사용자 지시).

## 무엇을 고정하는가

이 도구가 세우는 확증 방법 자체가 맞는지를 실기 로그 없이도 검증할 수
있어야 한다 — 합성 로그로 "명백히 인과관계가 있는 경우"와 "명백히 없는
경우"를 만들어, 도구가 그 둘을 실제로 구분하는지 고정한다."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import correlate_arm_motor_stall as tool  # noqa: E402


def _line_motor(ts: float, idle_s: float) -> str:
    return (f"[{ts:.3f}] [motor_watchdog] {idle_s:.2f}초 동안 새 모터 명령이 "
           f"없습니다 — 0속도를 강제 전송합니다\n")


def _line_write_timeout(ts: float, servo: int = 6) -> str:
    return f"[{ts:.3f}] [driver] WRITE_TIMEOUT servo={servo} — timed out\n"


# ── 파싱 ──────────────────────────────────────────────────────────────────


def test_두_종류의_줄을_각각_뽑아낸다(tmp_path):
    log = tmp_path / "bringup.log"
    log.write_text(
        _line_motor(100.5, 0.62)
        + "[100.10] [some_other_node] 무관한 줄\n"
        + _line_write_timeout(99.9)
    )

    motor_events, write_timeouts = tool._parse(str(log))

    assert len(motor_events) == 1
    assert motor_events[0].ts == 100.5
    assert motor_events[0].idle_s == 0.62
    assert write_timeouts == [99.9]


def test_무관한_줄은_무시한다(tmp_path):
    log = tmp_path / "bringup.log"
    log.write_text("[INFO] [1234.0] [mission_orchestrator]: 평범한 로그\n"
                   "완전히 다른 형식의 줄\n")

    motor_events, write_timeouts = tool._parse(str(log))

    assert motor_events == []
    assert write_timeouts == []


# ── 대조 ──────────────────────────────────────────────────────────────────


def test_정지_구간_안의_write_timeout은_확증된다():
    """모터 워치독이 ts=100.5, idle=0.62초 -> 정지 시작 추정 99.88.
    write_timeout이 99.9(그 구간 안)에 있으면 확증돼야 한다."""
    events = [tool.MotorStallEvent(ts=100.5, idle_s=0.62, stall_start=99.88)]
    write_timeouts = [99.9]

    matched, unmatched, orphans = tool.correlate(events, write_timeouts)

    assert len(matched) == 1
    assert matched[0][1] == 99.9
    assert unmatched == []
    assert orphans == []


def test_정지_구간과_동떨어진_write_timeout은_확증되지_않는다():
    """write_timeout이 워치독 발동보다 한참 전(무관한 사건)이면 엮이면 안 된다."""
    events = [tool.MotorStallEvent(ts=100.5, idle_s=0.62, stall_start=99.88)]
    write_timeouts = [50.0]

    matched, unmatched, orphans = tool.correlate(events, write_timeouts)

    assert matched == []
    assert len(unmatched) == 1
    assert orphans == [50.0]


def test_window을_넘어서면_확증되지_않는다():
    """정지 구간 경계 바로 밖(window 기본 0.2초)이면 우연으로 본다."""
    events = [tool.MotorStallEvent(ts=100.5, idle_s=0.62, stall_start=99.88)]
    # 구간은 [99.68, 100.7] (window=0.2) — 99.4는 그 밖이다.
    write_timeouts = [99.4]

    matched, unmatched, orphans = tool.correlate(events, write_timeouts, window_s=0.2)

    assert matched == []
    assert len(unmatched) == 1


def test_write_timeout_하나가_두_모터_이벤트에_중복으로_안_쓰인다():
    """같은 write_timeout을 두 번 "확증"으로 세면 비율이 부풀려진다."""
    events = [
        tool.MotorStallEvent(ts=100.5, idle_s=0.62, stall_start=99.88),
        tool.MotorStallEvent(ts=100.6, idle_s=0.70, stall_start=99.90),
    ]
    write_timeouts = [99.9]  # 두 구간 모두에 들어간다

    matched, unmatched, orphans = tool.correlate(events, write_timeouts)

    assert len(matched) == 1
    assert len(unmatched) == 1
    assert orphans == []


def test_모든_모터_이벤트가_확증되면_orphan이_없다():
    events = [tool.MotorStallEvent(ts=10.0, idle_s=0.5, stall_start=9.5)]
    write_timeouts = [9.6]

    matched, unmatched, orphans = tool.correlate(events, write_timeouts)

    assert len(matched) == 1
    assert orphans == []


# ── CLI 전체 ─────────────────────────────────────────────────────────────


def test_실기_증상_재현_로그를_돌리면_확증된다(tmp_path, monkeypatch, capsys):
    """이 세션에서 세운 가설을 그대로 재현한 합성 로그 — write_timeout이
    걸린 순간부터 모터 워치독이 idle_s초 뒤에 강제 개입한다. CLI 진입점
    (main())을 그대로 통과시켜 argparse 배선까지 같이 고정한다."""
    log = tmp_path / "bringup.log"
    write_ts = 1000.000
    idle_s = 0.55
    motor_ts = write_ts + idle_s
    log.write_text(_line_write_timeout(write_ts) + _line_motor(motor_ts, idle_s))
    monkeypatch.setattr(sys, "argv", ["correlate_arm_motor_stall.py", str(log)])

    rc = tool.main()

    out = capsys.readouterr().out
    assert rc == 0
    assert "확증됨" in out and "1/1건 (100%)" in out
    assert "가설" in out and "뒷받침된다" in out


def test_로그_파일이_없으면_에러_코드를_돌려준다(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["correlate_arm_motor_stall.py",
                                      "/no/such/file.log"])

    rc = tool.main()

    assert rc == 1
    assert "찾을 수 없다" in capsys.readouterr().err


def test_모터_워치독이_없으면_증상_자체가_없다고_알린다(tmp_path, monkeypatch, capsys):
    log = tmp_path / "bringup.log"
    log.write_text(_line_write_timeout(1.0))  # write_timeout은 있지만 모터 워치독은 없음
    monkeypatch.setattr(sys, "argv", ["correlate_arm_motor_stall.py", str(log)])

    rc = tool.main()

    assert rc == 0
    assert "증상 자체가" in capsys.readouterr().out
