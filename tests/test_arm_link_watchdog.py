"""그리퍼/팔 버스 통신 워치독 (2026-09-06 — 벤더 시리얼 write_timeout 결함 후속).

## 무엇을 고정하는가

third_party/soarm_provided_d/soarm_lab/driver_sdk.py가 write_timeout 없이
시리얼을 열고 있어서, 쓰기가 한 번 걸리면 그 클래스의 유일한 락이 다시는
안 풀리고 그리퍼든 팔 관절이든 이 버스의 모든 서보 통신이 영원히 막혔다
(같은 날 고쳤다 — test_arm_bus_write_timeout.py 참고). 하지만 그 고침만으로
"버스가 여전히 응답을 안 준다" 자체가 사라지는 것은 아니다 — CARRY 상태는
매 사이클 `get_load()`를 부르는데, 이게 실패하면 그 사이클의
`apply_velocity()` 호출이 늦어져 STM32 쪽 바퀴 모터 워치독(0.5초)이 대신
걸린다. 여기서 고정하는 성질은 base_liveness.py의 LivenessLatch와 같다 —
**증상(바퀴가 안 돈다)이 아니라 원인(그리퍼 버스)을 Host에게 말한다.**"""

import threading

from domain.adapters.fake.fake_arm import LOAD_HOLDING, FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import HostCommand, MissionState, Report
from domain.task.baseline_mission import (
    ArmLinkWatchdog,
    BaselineCarryState,
    BaselinePorts,
)

# ArmDriver.get_load()의 포트 계약상 "읽기 실패" 신호 — None이 아니라 이
# 음수값이다(ros2_arm_driver.LOAD_UNKNOWN과 같은 값, 세 계층이 독립적으로
# 정의하는 관례라 여기서도 그대로 import하지 않는다).
LOAD_READ_FAILED = -1.0


# ── 판정 자체 ──────────────────────────────────────────────────────────────


def test_한두_번_실패는_보고하지_않는다():
    """일회성 통신 잡음까지 알리면 그것대로 로그가 찬다."""
    watchdog = ArmLinkWatchdog(threshold=3)
    results = [watchdog.observe(LOAD_READ_FAILED) for _ in range(2)]
    assert results == [None, None]


def test_threshold번_연속_실패하면_한_번_보고한다():
    watchdog = ArmLinkWatchdog(threshold=3)
    results = [watchdog.observe(LOAD_READ_FAILED) for _ in range(5)]
    reported = [r for r in results if r is not None]
    assert len(reported) == 1
    assert "3회 연속 실패" in reported[0]


def test_복구도_한_번_말한다():
    """고장만 말하고 복구를 안 말하면, 사람이 아직 고장 중인 줄 안다."""
    watchdog = ArmLinkWatchdog(threshold=3)
    for _ in range(3):
        watchdog.observe(LOAD_READ_FAILED)

    message = watchdog.observe(LOAD_HOLDING)

    assert message is not None and "복구" in message


def test_실패_사이에_성공이_끼면_카운터가_리셋된다():
    """2번 실패, 1번 성공, 다시 2번 실패 — 연속 3번을 채운 적이 없다."""
    watchdog = ArmLinkWatchdog(threshold=3)
    sequence = [LOAD_READ_FAILED, LOAD_READ_FAILED, LOAD_HOLDING,
                LOAD_READ_FAILED, LOAD_READ_FAILED]

    results = [watchdog.observe(v) for v in sequence]

    assert all(r is None for r in results)


def test_정상_읽기만_이어지면_아무_말도_안_한다():
    watchdog = ArmLinkWatchdog(threshold=3)
    results = [watchdog.observe(LOAD_HOLDING) for _ in range(10)]
    assert all(r is None for r in results)


# ── CARRY 상태와의 결합 ─────────────────────────────────────────────────────


def _ports(arm):
    return BaselinePorts(
        base=FakeBase(),
        arm=arm,
        perception=ScriptedPerception(),
        host=FakeHostLink([HostCommand(state=MissionState.CARRY, linear_x=0.05)]),
        lidar=FakeLidar(),
        estop=threading.Event(),
    )


def test_CARRY가_그리퍼_읽기_실패를_Host에_보고한다():
    """그날 실기 증상 재현 — 바퀴가 안 도는 것처럼 보여도 원인이 그리퍼임을
    Host가 알아야 사람이 바퀴 쪽을 의심하지 않는다."""
    ports = _ports(FakeArm(load_ratio=[LOAD_READ_FAILED] * 5))
    state = BaselineCarryState("rook")

    for _ in range(5):
        state = state.execute(ports)

    assert Report.ARM_LINK_DEGRADED in ports.host.reported_kinds


def test_연속_실패가_이어져도_보고는_한_번뿐이다():
    """워치독 거부와 같은 이유 — 매 사이클 보고하면 로그가 이것으로 가득 찬다."""
    ports = _ports(FakeArm(load_ratio=[LOAD_READ_FAILED] * 20))
    state = BaselineCarryState("rook")

    for _ in range(20):
        state = state.execute(ports)

    degraded = [k for k in ports.host.reported_kinds
                if k == Report.ARM_LINK_DEGRADED]
    assert len(degraded) == 1


def test_실패_후_복구되면_그것도_한_번_보고한다():
    ports = _ports(FakeArm(load_ratio=[
        LOAD_READ_FAILED, LOAD_READ_FAILED, LOAD_READ_FAILED,
        LOAD_HOLDING, LOAD_HOLDING,
    ]))
    state = BaselineCarryState("rook")

    for _ in range(5):
        state = state.execute(ports)

    degraded_count = ports.host.reported_kinds.count(Report.ARM_LINK_DEGRADED)
    assert degraded_count == 2  # 발생 1회 + 복구 1회


def test_정상_그리퍼는_경보를_내지_않는다():
    """FakeArm 기본값(LOAD_HOLDING)은 정상 읽기다 — 우연히 경보가 나면
    실기에서 매 CARRY마다 거짓 경보가 뜬다."""
    ports = _ports(FakeArm(load_ratio=LOAD_HOLDING))
    state = BaselineCarryState("rook")

    for _ in range(30):
        state = state.execute(ports)

    assert Report.ARM_LINK_DEGRADED not in ports.host.reported_kinds
