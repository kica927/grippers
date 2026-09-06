"""Host가 보낸 속도 명령을 Pi가 실제로 낼 속도로 바꾼다 (팀 확정, 2026-08-26).

순수 계산이다 — 포트도 ROS도 모른다. 주행 안전의 마지막 한 겹이라 단위
테스트로 고정해 둘 수 있어야 한다.

## 왜 Pi가 자르는가

Host가 좌표와 경로를 소유하지만, **바퀴를 실제로 돌리는 것은 Pi다.** 속도
한계는 그 한계를 어길 수 있는 쪽이 아니라 물리적으로 집행할 수 있는 쪽에
있어야 한다. Host의 버그나 패킷 손상으로 0.1이 1.0으로 오더라도 Pi가 잘라
낸다. Pi가 명령을 **고르지는** 않는다 — 방향은 Host 것 그대로 두고 크기만
합의된 값으로 제한한다.

## 제자리회전은 정말 제자리여야 한다

팀이 합의한 명령 어휘는 직진·수평이동·제자리회전·제자리정지 네 가지다.
"제자리"회전에 병진이 섞여 들어오면 그것은 합의된 네 가지 중 무엇도 아니다.
추측해서 둘 중 하나를 골라 실행하는 대신 **거부하고 Host에 되돌려준다** —
이 저장소의 "모르면 실패" 관례 그대로다.

직진과 수평이동이 함께 오는 것은 막지 않는다. 메카넘 베이스에서 그 둘은
자연스러운 한 동작(대각선 이동)이고, "제자리"라는 단서가 붙은 쪽은 회전뿐이다.
"""

import math
from dataclasses import dataclass

# 팀 합의 속도 (2026-08-26). 직진과 수평이동이 같은 값이다.
#
# ⚠️ 이 베이스에는 데드밴드가 있다 — 0.05 m/s 명령에는 바퀴가 아예 안 돈다
# (2026-08-24 실기, tools/grasp_test_console.py의 APPROACH_SPEED_MPS 주석).
# 합의된 0.1은 그 위라 실제로 움직인다. 더 느리게 가야 한다면 속도를 낮추지
# 말고 짧은 버스트와 정지를 반복할 것 — 데드밴드 아래 속도는 아무리 오래
# 줘도 안 움직이는데 /odom_raw는 움직였다고 보고한다.
AGREED_LINEAR_MPS = 0.1
AGREED_ROTATION_RAD_S = 0.25

# 바구니 최종 접근에서만 쓰는 더 낮은 상한 (2026-08-26 사용자 승인).
#
# ## 왜 구간을 나누는가 — 지연이 허용폭보다 크다
#
# Host가 "지금 멈춰"라고 판단한 순간부터 바퀴가 실제로 서기까지 지연이 쌓인다.
#
#     Host 루프 한 바퀴(8Hz)    125 ms
#     UDP + Pi 수신              10 ms
#     Pi 사이클(10Hz)           100 ms
#     ------------------------------
#     합계                      235 ms
#
# 0.1 m/s면 그동안 **23.5 mm**를 더 간다. 그런데 INSERT 허용폭은 ±15 mm
# (BASKET_STOP_TOLERANCE_M)다 — **오버슈트가 창보다 크다.** 창 안에 우연히
# 들어갈 수는 있어도 제어되는 것이 아니다.
#
# 0.06으로 낮추면 같은 지연이 14 mm가 되어 창 안에 들어온다.
#
# ## 왜 더 낮추지 못하는가
#
# 데드밴드가 0.05다. 그 아래는 아무리 오래 줘도 안 움직인다. 0.06이 실제로
# 도는 최저 속도이므로 여기가 바닥이다. 더 잘게 가야 하면 속도가 아니라
# **끊어 가기**로 해야 한다(ros2_mecanum_base.creep_forward).
#
# ## 왜 Pi가 자르는가 — 이건 경로가 아니라 센서 제약이다
#
# 차량 제어는 Host 소유다. 그런데 이 상한은 "어디로 갈지"가 아니라 "이보다
# 빠르면 내 센서로 판정 자체가 불가능하다"는 Pi 쪽 사실이다. 데드밴드나
# 바구니 절벽과 같은 성격이라 Pi가 지킨다 — Host가 0.1을 보내도 이 구간에서는
# 0.06으로 실행되고, 그 사실을 보고에 적는다.
BASKET_APPROACH_MPS = 0.06

# RETURN_HOME 전용 상향 상한 (2026-09-06, 사용자 지시 — "도전이긴 한데
# 시간을 조금만 줄일 수 있을까"). RETURN_HOME은 기물을 포기했거나 하나를
# 다 옮긴 뒤 mcfg.DEFAULT_HOME_XY로 돌아가기만 하는 구간이라, GRASP/INSERT
# 처럼 정밀 판정에 걸리는 지연 민감도가 없다 — 그래서 이 구간에서만 더
# 빠르게 달려도 된다.
#
# 값 선정 근거:
#   RETURN_HOME_LINEAR_MPS=0.2 — odom_publisher_node.app_cmd_vel_callback의
#     하드 캡(0.2 m/s)과 정확히 같다. 그 위로는 Pi가 어차피 다시 잘라서
#     의미가 없다.
#   RETURN_HOME_ROTATION_RAD_S=0.3 — 순수 회전 데드밴드 펄싱의 문턱인
#     ROTATE_BURST_SPEED_RAD_S(0.4, ros2_mecanum_base.py)보다 여전히
#     작다 — 즉 이 값을 올려도 펄싱 경로(껐다 켰다 하는 버스트)가 그대로
#     유지되고, duty만 0.25/0.4=0.625에서 0.3/0.4=0.75로 조금 늘 뿐이다.
#     0.4 이상으로 올렸다면 펄싱을 완전히 우회해 다른 동작이 됐을
#     것이므로 일부러 그 아래로 잡았다.
#
# ⚠️ 실기 미검증이다 — 회전이 빨라지면 오버슈트 폭도 커져서 yaw 헌팅
# (mission_config.ROTATE_OSCILLATION_TOGGLE_LIMIT 참고)이 오히려 심해질
# 가능성이 있다. 다음 실기에서 RETURN_HOME 구간의 회전 왕복이 늘었는지
# 반드시 확인할 것.
#
# ⚠️ 2026-09-06 실기 확인 — 이 상수들 자체는 문제가 아니었다. 아래
# `_HOST_STATE_SPEED_OVERRIDE`를 **상한(clamp 캡)**으로만 처음 구현했었는데,
# Host의 encode()는 이 구간에서도 항상 AGREED_LINEAR_MPS/AGREED_ROTATION_
# RAD_S 고정값만 실어 보낸다(vehicle_link.encode()의 네 가지 동작 참고,
# RETURN_HOME이라고 더 큰 값을 보내는 분기가 없다). `_clamp`는 "값이 캡보다
# 크면 캡으로 자르는" 함수라 min(0.25, 0.3)=0.25로, 캡을 아무리 올려도
# **원래 값 밑으로는 절대 안 올라간다.** 실기 로그(apply_velocity 인자)에서
# RETURN_HOME 내내 각속도가 그대로 0.25로 나가는 것으로 확증했다 — 사용자가
# "속도 별 차이 없다"고 보고한 그대로였다.
#
# 사용자가 애초에 제안한 설계("host가 RETURN_HOME을 보내면 pi에서 속도에
# 배수를 준다")를 그대로 따랐어야 했다 — 아래 resolve_motion()은 이제 캡이
# 아니라 **배수**로 스케일업한 뒤, 그 결과를 다시 캡으로 한 번 더 잘라
# 안전판을 유지한다.
RETURN_HOME_LINEAR_MPS = 0.2
RETURN_HOME_ROTATION_RAD_S = 0.3

# APPROACH_PIECE/CARRY_TO_DEST 전용 상향 상한 (2026-09-06, 사용자 지시 —
# "APPROACH_PIECE/CARRY_TO_DEST 정도만 전진 0.18, 회전 0.28로 상향").
# RETURN_HOME(0.2/0.3)보다 보수적으로 잡은 값이다 — 이 두 구간은
# RETURN_HOME과 달리 정밀 판정이 걸려 있다: APPROACH_PIECE는 GRASP 트리거
# 거리 판정을, CARRY_TO_DEST는 물체를 든 채 이동(파지력이 아니라 흔들림이
# 원인일 수 있는 낙하 사고, 2026-09-06 knight 2회)을 각각 겪는다. GRASP_
# ALIGN/GRASP_REPLAN(같은 호스트 어휘로 APPROACH에 매핑되지만 원본 이름은
# 다르다)과 FACE_BOX/NUDGE_BOX/PLACE(바구니 근접 정렬)는 대상이 아니다 —
# host_state로 구분하므로 정확히 이 두 이름만 걸린다.
#
# 실기 확인(2026-09-06, rook/knight/queen 3구간 전부 성공, 구동계·그리퍼
# 경보 없음) 직후 사용자가 "좀 빠르네"라며 선속만 0.18 -> 0.14로 낮췄다.
# 회전(0.28)은 그대로 둔다 — 사용자가 선속만 지목했다.
#
# 0.14 < Pi 하드캡(0.2, odom_publisher_node.app_cmd_vel_callback) — 하드캡을
# 넘겨 봐야 조용히 잘려 상수가 무의미해지는 일이 없다. 데드밴드(0.05,
# AGREED_LINEAR_MPS 주석 참고)보다는 위라 실제로 돈다.
# 0.28 < 데드밴드 펄싱 문턱(0.4, ROTATE_BURST_SPEED_RAD_S) — 순수 회전이
# 펄싱 경로를 벗어나지 않는다.
#
# ⚠️ 회전(0.28)과 CARRY_TO_DEST 구간 자체는 여전히 실기 1회 검증이다 —
# knight 낙하가 이번엔 없었지만(그리퍼 토크 상향과 겹친 실행이라 어느 쪽
# 효과인지 아직 못 가른다), 계속 지켜볼 것.
APPROACH_CARRY_LINEAR_MPS = 0.14
APPROACH_CARRY_ROTATION_RAD_S = 0.28

# HostCommand.host_state(Host 원본 FSM 상태 이름)가 이 표에 있으면 그
# (linear_cap, angular_cap) 쌍을 쓴다. 표에 없거나 host_state가 빈 문자열
# (구버전 Host와의 하위호환)이면 기본값(AGREED_LINEAR_MPS/ROTATION_RAD_S)
# 그대로 — "모르면 원래 값" 관례. HostCommand.host_state 정의부(순환
# import를 피하려고 여기서 문자열을 직접 적는다) 참고.
_HOST_STATE_SPEED_OVERRIDE = {
    "RETURN_HOME": (RETURN_HOME_LINEAR_MPS, RETURN_HOME_ROTATION_RAD_S),
    "APPROACH_PIECE": (APPROACH_CARRY_LINEAR_MPS, APPROACH_CARRY_ROTATION_RAD_S),
    "CARRY_TO_DEST": (APPROACH_CARRY_LINEAR_MPS, APPROACH_CARRY_ROTATION_RAD_S),
}

# 부동소수 잡음을 0으로 본다. UDP+JSON을 거치며 0.0이 1e-17로 오는 경우가
# 있는데, 그걸 "회전 명령"으로 읽으면 병진과 섞였다고 오판해 거부한다.
EPSILON = 1e-6

# MissionState.APPROACH_BOX와 같은 문자열이어야 한다. baseline_ports를
# import하면 순환이 되므로 값을 직접 적고, 테스트가 두 값을 대조한다.
_APPROACH_BOX = "APPROACH_BOX"


@dataclass(frozen=True)
class Motion:
    """실제로 베이스에 낼 속도."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_stop(self) -> bool:
        return (abs(self.linear_x) < EPSILON
                and abs(self.linear_y) < EPSILON
                and abs(self.angular_z) < EPSILON)


STOP = Motion()


@dataclass(frozen=True)
class MotionDecision:
    """`resolve_motion`의 결과. 거부됐으면 `ok=False`이고 `motion`은 정지다."""

    ok: bool
    motion: Motion
    reason: str = ""


def _clamp(value: float, limit: float) -> float:
    """부호는 두고 크기만 limit로 자른다.

    비유한값(NaN/Inf)은 **정지(0.0)로 본다.** 손상·조작된 Host 명령이 NaN 을
    실어 오면 `abs(nan) < EPSILON` 도 `min(nan, limit)` 도 nan 을 그대로
    돌려줘 베이스까지 샜다(RoboSec F1, 2026-08-30 실기 확증). 쓰레기 수치가
    최대속도가 되어서도 안 되므로 Inf 도 같이 정지로 접는다 — fail-safe.
    """
    if not math.isfinite(value):
        return 0.0
    if abs(value) < EPSILON:
        return 0.0
    return math.copysign(min(abs(value), limit), value)


def _scale_and_clamp(value: float, agreed: float, cap: float) -> float:
    """`value`(항상 0 아니면 ±agreed 둘 중 하나 — Host encode()의 이산
    어휘 참고)를 cap 크기로 비례 확대한 뒤 다시 cap으로 자른다.

    `_clamp`와 다른 점: `_clamp(0.25, 0.3)`은 0.25를 그대로 돌려준다(캡을
    올려도 원래 값 밑으로는 못 올라간다) — RETURN_HOME 상향이 실기에서
    아무 효과가 없었던 그 버그다. 여기서는 대신 (cap/agreed) 배수를 먼저
    곱해 0.25 -> 0.3으로 실제로 올린다. agreed가 0이면(설정 실수) 배수
    계산이 무의미하므로 안전하게 그냥 캡으로 자른다."""
    if agreed <= 0.0:
        return _clamp(value, cap)
    return _clamp(value * (cap / agreed), cap)


def resolve_motion(command) -> MotionDecision:
    """`HostCommand`를 실제 속도로 바꾼다.

    우선순위:
      1. `stop=True`면 나머지를 보지 않고 정지한다. 제자리정지가 가장 센
         명령이어야 한다 — 정지 의도가 다른 필드의 잔여값에 지면 안 된다.
      2. 제자리회전에 병진이 섞였으면 거부한다(정지 + 사유).
      3. 나머지는 합의된 크기로 자른다.
    """
    if command is None:
        return MotionDecision(False, STOP, "명령 없음")

    if command.stop:
        return MotionDecision(True, STOP, "제자리정지")

    rotating = abs(command.angular_z) >= EPSILON
    translating = (abs(command.linear_x) >= EPSILON
                   or abs(command.linear_y) >= EPSILON)
    if rotating and translating:
        return MotionDecision(
            False, STOP,
            "제자리회전에 병진이 섞였다 — "
            f"linear=({command.linear_x:.3f}, {command.linear_y:.3f}), "
            f"angular={command.angular_z:.3f}")

    # 바구니로 붙는 구간만 더 낮은 상한을 쓴다. 회전은 안 낮춘다 — 회전은
    # 한 사이클에 1.8도라 이미 허용치(5도)의 3분의 1이다.
    #
    # 2026-09-06: RETURN_HOME은 반대로 **올린다** — command.state(Host·Pi
    # 합의 어휘)가 아니라 command.host_state(Host 원본 상태 이름)로 판단한다.
    # 이 둘은 절대 동시에 안 걸린다 — RETURN_HOME일 때 state는 항상
    # APPROACH지 APPROACH_BOX가 아니다. 그래도 바구니 저속 캡을 if/elif로
    # 먼저 두어, 혹시 표가 잘못 채워지는 미래의 사고에도 "안전 쪽(저속)"이
    # 항상 이기게 한다.
    linear_cap, angular_cap = AGREED_LINEAR_MPS, AGREED_ROTATION_RAD_S
    boosted = False
    if command.state == _APPROACH_BOX:
        linear_cap = BASKET_APPROACH_MPS
    elif command.host_state in _HOST_STATE_SPEED_OVERRIDE:
        linear_cap, angular_cap = _HOST_STATE_SPEED_OVERRIDE[command.host_state]
        boosted = True

    if boosted:
        # Host가 이 필드들에 싣는 크기는 항상 0 아니면 AGREED_* 고정값
        # 하나뿐이다(encode() 참고) — clamp만으로는 상한을 올려도 그
        # 고정값 밑으로 못 올라가므로, 여기서만 배수로 실제로 밀어올린다.
        motion = Motion(
            linear_x=_scale_and_clamp(command.linear_x, AGREED_LINEAR_MPS, linear_cap),
            linear_y=_scale_and_clamp(command.linear_y, AGREED_LINEAR_MPS, linear_cap),
            angular_z=_scale_and_clamp(command.angular_z, AGREED_ROTATION_RAD_S, angular_cap),
        )
    else:
        motion = Motion(
            linear_x=_clamp(command.linear_x, linear_cap),
            linear_y=_clamp(command.linear_y, linear_cap),
            angular_z=_clamp(command.angular_z, angular_cap),
        )
    slowed = (linear_cap < AGREED_LINEAR_MPS
              and (abs(command.linear_x) > linear_cap + EPSILON
                   or abs(command.linear_y) > linear_cap + EPSILON))
    if slowed:
        return MotionDecision(
            True, motion,
            f"바구니 접근 구간이라 {linear_cap:.2f} m/s로 낮췄다 "
            f"(명령 {max(abs(command.linear_x), abs(command.linear_y)):.2f})")
    if linear_cap > AGREED_LINEAR_MPS or angular_cap > AGREED_ROTATION_RAD_S:
        return MotionDecision(
            True, motion,
            f"{command.host_state} 구간이라 속도를 올렸다 "
            f"(선속 상한 {linear_cap:.2f} m/s, 각속 상한 {angular_cap:.2f} rad/s)")
    return MotionDecision(True, motion)
