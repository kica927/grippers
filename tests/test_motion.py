"""Host 속도 명령 해석의 계약을 고정한다 (팀 확정, 2026-08-26).

주행 안전의 마지막 한 겹이다. 여기서 새는 값은 그대로 바퀴로 간다."""

import pytest

from domain.task import motion as mo
from domain.ports.baseline_ports import HostCommand, MissionState
from domain.task.motion import (
    AGREED_LINEAR_MPS,
    AGREED_ROTATION_RAD_S,
    resolve_motion,
)


def _command(**kwargs):
    return HostCommand(state=MissionState.APPROACH, **kwargs)


def test_합의된_속도는_그대로_통과한다():
    decision = resolve_motion(_command(linear_x=AGREED_LINEAR_MPS))

    assert decision.ok
    assert decision.motion.linear_x == AGREED_LINEAR_MPS


@pytest.mark.parametrize("sent,expected", [
    (1.0, AGREED_LINEAR_MPS),
    (-1.0, -AGREED_LINEAR_MPS),
    (0.05, 0.05),          # 합의보다 느린 것은 자르지 않는다
])
def test_직진은_크기만_자르고_부호는_지킨다(sent, expected):
    decision = resolve_motion(_command(linear_x=sent))

    assert decision.ok
    assert decision.motion.linear_x == pytest.approx(expected)


def test_수평이동도_같은_한계를_쓴다():
    """팀 합의: linear.x와 linear.y가 같은 0.1이다."""
    decision = resolve_motion(_command(linear_y=-5.0))

    assert decision.motion.linear_y == -AGREED_LINEAR_MPS


def test_제자리회전은_별도_한계를_쓴다():
    decision = resolve_motion(_command(angular_z=3.0))

    assert decision.motion.angular_z == AGREED_ROTATION_RAD_S


def test_제자리정지가_다른_모든_필드를_이긴다():
    """정지 의도가 다른 필드의 잔여값에 지면 안 된다."""
    decision = resolve_motion(
        _command(linear_x=0.1, linear_y=0.1, angular_z=0.25, stop=True))

    assert decision.ok
    assert decision.motion.is_stop


def test_직진과_수평이동은_함께_와도_된다():
    """메카넘에서 대각선 이동은 한 동작이다 — '제자리'라는 단서가 붙은 건 회전뿐."""
    decision = resolve_motion(_command(linear_x=0.1, linear_y=0.1))

    assert decision.ok
    assert decision.motion.linear_x == AGREED_LINEAR_MPS
    assert decision.motion.linear_y == AGREED_LINEAR_MPS


def test_제자리회전에_병진이_섞이면_거부한다():
    """합의된 네 가지 명령 중 무엇도 아니다 — 추측해 실행하지 않는다."""
    decision = resolve_motion(_command(linear_x=0.1, angular_z=0.25))

    assert not decision.ok
    assert decision.motion.is_stop
    assert "제자리회전" in decision.reason


def test_부동소수_잡음은_회전으로_읽지_않는다():
    """UDP+JSON을 거치며 0.0이 1e-17로 오는 경우가 있다."""
    decision = resolve_motion(_command(linear_x=0.1, angular_z=1e-17))

    assert decision.ok
    assert decision.motion.angular_z == 0.0


def test_명령이_없으면_거부하고_정지한다():
    decision = resolve_motion(None)

    assert not decision.ok
    assert decision.motion.is_stop


# ── 바구니 접근 구간 속도 상한 (2026-08-26) ────────────────────────────────
# 지연이 허용폭보다 크다는 계산이 근거다. Host가 "멈춰"라고 판단한 순간부터
# 바퀴가 실제로 서기까지 235ms가 쌓이는데(Host 루프 125 + 링크 10 + Pi
# 사이클 100), 0.1 m/s면 그동안 23.5mm를 더 간다. INSERT 허용폭은 ±15mm다.

def test_APPROACH_BOX에서는_더_느리게_자른다():
    """0.1로 오면 0.06으로 실행된다 — 그러지 않으면 오버슈트가 창을 넘는다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=0.1))

    assert decision.ok
    assert decision.motion.linear_x == pytest.approx(mo.BASKET_APPROACH_MPS)


def test_낮췄다는_사실을_사유에_남긴다():
    """Host는 0.1을 보냈는데 0.06으로 도는 것을 모르면 안 된다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=0.1))

    assert "낮췄다" in decision.reason


def test_이미_느리면_그대로_둔다():
    """Host가 스스로 0.06을 보냈으면 손대지 않고, 사유도 안 남긴다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=0.06))

    assert decision.motion.linear_x == pytest.approx(0.06)
    assert decision.reason == ""


def test_다른_상태는_안_낮춘다():
    """주행 구간까지 느리게 하면 시연이 하염없이 길어진다."""
    for state in (MissionState.APPROACH, MissionState.CARRY):
        decision = resolve_motion(HostCommand(state=state, linear_x=0.1))

        assert decision.motion.linear_x == pytest.approx(mo.AGREED_LINEAR_MPS)


def test_회전은_안_낮춘다():
    """회전은 한 사이클에 1.8도라 이미 허용치(5도)의 3분의 1이다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, angular_z=0.25))

    assert decision.motion.angular_z == pytest.approx(mo.AGREED_ROTATION_RAD_S)


def test_상한이_데드밴드_위에_있다():
    """0.05 아래는 아무리 오래 줘도 안 움직인다 — 낮출 수 있는 바닥이다."""
    assert mo.BASKET_APPROACH_MPS > 0.05


def test_상태_문자열이_포트와_같다():
    """motion은 순환 import를 피하려고 문자열을 직접 들고 있다. 둘이
    갈라지면 상한이 조용히 안 걸린다."""
    assert mo._APPROACH_BOX == MissionState.APPROACH_BOX


# ── RETURN_HOME 구간 속도 상향 (2026-09-06) ────────────────────────────────
# host_state는 command.state(Host·Pi 합의 어휘, IDLE/APPROACH/...)와 다른
# 채널이다 — RETURN_HOME은 state로는 항상 APPROACH로 온다(vehicle_link.
# _STATE_TO_PI 참고). 여기서는 그 압축 전 원본 이름(host_state)만으로 상한을
# 올린다.
#
# ⚠️ 2026-09-06 실기 확인 — 아래 두 테스트는 원래 linear_x=1.0/angular_z=1.0
# 같은 "상한보다 큰" 인위적인 값으로만 검증했다. 그래서 resolve_motion()을
# 처음에 clamp 캡 방식으로 짰을 때도 둘 다 통과했는데, 실기에서는 전혀 안
# 올랐다 — Host가 실제로 보내는 값은 항상 AGREED_LINEAR_MPS/AGREED_ROTATION_
# RAD_S 고정값(vehicle_link.encode() 참고)이고, clamp는 "그 값이 캡보다 크면
# 자르는" 함수라 값이 이미 캡보다 작으면 캡을 올려도 그대로다. 이 테스트들은
# "클램프가 최종 안전판으로는 작동한다"만 확인했을 뿐 "실제로 빨라지는가"는
# 확인하지 못했다 — 그래서 바로 아래에 Host의 실제 고정값으로 검증하는
# 테스트를 추가했다. 둘 다 남겨 둔다: 이것들은 안전판(상한 자체가 못 넘어감),
# 아래 새 것들은 효과(실제로 올라감)를 각각 고정한다.

def test_RETURN_HOME이면_직진_상한이_오른다():
    """캡 자체가 RETURN_HOME_LINEAR_MPS를 넘지 않는다는 안전판 확인."""
    decision = resolve_motion(
        _command(linear_x=1.0, host_state="RETURN_HOME"))

    assert decision.ok
    assert decision.motion.linear_x == pytest.approx(mo.RETURN_HOME_LINEAR_MPS)


def test_RETURN_HOME이면_회전_상한도_오른다():
    """캡 자체가 RETURN_HOME_ROTATION_RAD_S를 넘지 않는다는 안전판 확인."""
    decision = resolve_motion(
        _command(angular_z=1.0, host_state="RETURN_HOME"))

    assert decision.motion.angular_z == pytest.approx(mo.RETURN_HOME_ROTATION_RAD_S)


def test_Host의_실제_고정값도_RETURN_HOME에서_실제로_오른다():
    """진짜 회귀 테스트. Host는 이 구간에서도 AGREED_LINEAR_MPS/
    AGREED_ROTATION_RAD_S 고정값만 보낸다(vehicle_link.encode()가 RETURN_HOME
    이라고 다른 값을 보내는 분기가 없다) — 그러니 여기 값도 정확히 그
    고정값으로 줘야, "clamp 캡만 올리고 값 자체는 안 올리는" 예전 버그가
    되돌아오면 이 테스트가 바로 깨진다."""
    decision = resolve_motion(
        _command(linear_x=AGREED_LINEAR_MPS, host_state="RETURN_HOME"))

    assert decision.motion.linear_x == pytest.approx(mo.RETURN_HOME_LINEAR_MPS)


def test_Host의_실제_고정_회전값도_RETURN_HOME에서_실제로_오른다():
    decision = resolve_motion(
        _command(angular_z=AGREED_ROTATION_RAD_S, host_state="RETURN_HOME"))

    assert decision.motion.angular_z == pytest.approx(mo.RETURN_HOME_ROTATION_RAD_S)


def test_방향_부호는_배수를_곱해도_유지된다():
    decision = resolve_motion(
        _command(angular_z=-AGREED_ROTATION_RAD_S, host_state="RETURN_HOME"))

    assert decision.motion.angular_z == pytest.approx(-mo.RETURN_HOME_ROTATION_RAD_S)


def test_RETURN_HOME이_아니면_안_오른다():
    """host_state가 없거나(구버전 Host) 다른 값이면 기본 속도 그대로다."""
    for host_state in ("", "GRASP_ALIGN", "SEARCH_TARGET"):
        decision = resolve_motion(_command(linear_x=1.0, host_state=host_state))
        assert decision.motion.linear_x == pytest.approx(AGREED_LINEAR_MPS)


def test_올렸다는_사실을_사유에_남긴다():
    decision = resolve_motion(
        _command(linear_x=1.0, host_state="RETURN_HOME"))

    assert "RETURN_HOME" in decision.reason
    assert "올렸다" in decision.reason


def test_APPROACH_BOX_저속캡이_RETURN_HOME_상향보다_우선한다():
    """둘이 동시에 걸릴 일은 실제로 없지만(RETURN_HOME일 때 state는 항상
    APPROACH), 안전 쪽(저속)이 항상 이기게 방어적으로 짠 순서를 고정해 둔다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=1.0,
                    host_state="RETURN_HOME"))

    assert decision.motion.linear_x == pytest.approx(mo.BASKET_APPROACH_MPS)


def test_올린_속도도_데드밴드_펄싱_문턱_아래에_있다():
    """RETURN_HOME_ROTATION_RAD_S가 ROTATE_BURST_SPEED_RAD_S(0.4,
    ros2_mecanum_base.py) 이상이면 순수 회전이 펄싱 경로를 벗어나 전혀
    다른 동작이 된다 — 상수를 손대는 사람이 이 계약을 깨지 않게 고정한다."""
    assert mo.RETURN_HOME_ROTATION_RAD_S < 0.4


def test_올린_직진_속도가_Pi_하드캡_이내다():
    """odom_publisher_node.app_cmd_vel_callback의 linear.x 하드 캡(0.2)을
    넘으면 Pi가 다시 잘라내 이 상수가 조용히 무의미해진다."""
    assert mo.RETURN_HOME_LINEAR_MPS <= 0.2


# ── APPROACH_PIECE/CARRY_TO_DEST 속도 상향 (2026-09-06) ────────────────────
# RETURN_HOME과 같은 host_state 채널·같은 배수 메커니즘을 쓴다 — 다만 이
# 두 구간엔 정밀 판정이 걸려 있어 RETURN_HOME보다 보수적인 값을 쓴다
# (실기 1회 성공 확인 뒤 사용자가 선속만 0.18 -> 0.14로 낮췄다, mo.
# APPROACH_CARRY_LINEAR_MPS 정의부 참고). RETURN_HOME 절의 "Host의 실제
# 고정값도 실제로 오른다" 회귀 테스트와 같은 이유로, 여기도 AGREED_* 고정값
# 으로 검증한다 — 리터럴이 아니라 mo.APPROACH_CARRY_* 심볼을 참조하므로
# 값이 다시 바뀌어도 이 테스트들은 그대로 유효하다.

def test_APPROACH_PIECE에서_직진이_오른다():
    decision = resolve_motion(
        _command(linear_x=AGREED_LINEAR_MPS, host_state="APPROACH_PIECE"))

    assert decision.motion.linear_x == pytest.approx(mo.APPROACH_CARRY_LINEAR_MPS)


def test_APPROACH_PIECE에서_회전이_오른다():
    decision = resolve_motion(
        _command(angular_z=AGREED_ROTATION_RAD_S, host_state="APPROACH_PIECE"))

    assert decision.motion.angular_z == pytest.approx(mo.APPROACH_CARRY_ROTATION_RAD_S)


def test_CARRY_TO_DEST에서_직진이_오른다():
    decision = resolve_motion(
        _command(linear_x=AGREED_LINEAR_MPS, host_state="CARRY_TO_DEST"))

    assert decision.motion.linear_x == pytest.approx(mo.APPROACH_CARRY_LINEAR_MPS)


def test_CARRY_TO_DEST에서_회전이_오른다():
    decision = resolve_motion(
        _command(angular_z=AGREED_ROTATION_RAD_S, host_state="CARRY_TO_DEST"))

    assert decision.motion.angular_z == pytest.approx(mo.APPROACH_CARRY_ROTATION_RAD_S)


def test_GRASP_ALIGN은_같은_APPROACH_어휘여도_안_오른다():
    """GRASP_ALIGN/GRASP_REPLAN도 state는 APPROACH_PIECE와 똑같이 APPROACH로
    압축되지만(vehicle_link._STATE_TO_PI), host_state 원본 이름은 다르다 —
    이 상향이 압축된 state가 아니라 host_state로만 판단한다는 것의 확인."""
    for host_state in ("GRASP_ALIGN", "GRASP_REPLAN", "FACE_BOX", "NUDGE_BOX", "PLACE"):
        decision = resolve_motion(_command(linear_x=AGREED_LINEAR_MPS, host_state=host_state))
        assert decision.motion.linear_x == pytest.approx(AGREED_LINEAR_MPS)


def test_APPROACH_BOX_저속캡이_APPROACH_PIECE_상향보다_우선한다():
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=1.0,
                    host_state="APPROACH_PIECE"))

    assert decision.motion.linear_x == pytest.approx(mo.BASKET_APPROACH_MPS)


def test_APPROACH_CARRY_속도도_데드밴드_펄싱_문턱_아래에_있다():
    assert mo.APPROACH_CARRY_ROTATION_RAD_S < 0.4


def test_APPROACH_CARRY_직진_속도가_Pi_하드캡_이내다():
    assert mo.APPROACH_CARRY_LINEAR_MPS <= 0.2
