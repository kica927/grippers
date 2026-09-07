"""STM32 write_timeout + 모터 워치독 (2026-09-05, §2-6/§3-1 인수인계 후속).

## 배경

`ros_robot_controller_sdk.Board.buf_write()`가 모터·LED·부저·서보 등 모든
쓰기 명령의 유일한 통로다. 504531d가 여기서 예외가 나면 잡아서 로그만
남기게 고쳤지만("실패"는 잡는다), pyserial의 `Serial(...)`에 write_timeout
을 준 적이 없어서 **쓰기가 실패가 아니라 그냥 안 끝나고 계속 블록**되면
그 예외 자체가 안 났다. 이 노드는 기본 단일 스레드 rclpy.spin()이라, 그
블록 중엔 이 콜백도 다음 콜백(정지 명령 포함)도 전혀 못 돈다 — "정지
836회가 나갔는데 안 멈췄다"는 2026-08-28 사고와 같은 모양의 잠재 원인이다.

이 파일이 고정하는 것 둘:
1. `Board()`가 pyserial에 write_timeout을 실제로 넘긴다.
2. `set_motor_speed()`가 일정 시간 안 불리면(워치독 스레드가) 스스로
   0속도를 재전송한다 — 그 스레드는 rclpy 실행기와 무관한 독립 스레드라,
   실행기가 write() 블록으로 막혀 있어도 별개로 돈다(단, write_timeout이
   그 블록 자체를 짧게 끊어 주는 게 먼저다 — 이 두 조치는 서로를
   전제한다).

`Board`는 순수 pyserial 의존일 뿐 rclpy가 필요 없어서(모듈 상단 import
참고), 이 저장소의 다른 domain 테스트와 같은 방식으로 하드웨어·ROS2 없이
검증한다 — 진짜 시리얼 포트 대신 `_FakePort`로 대체한다.

## 2026-09-06 추가 — buf_write()의 재연결

위 둘만으로는 "모터 명령은 가는데 바퀴만 안 움직이고, 유일한 해결책이
재기동"이던 반복 신고를 다 못 막는다는 게 이날 드러났다. `recv_task`
(읽기 스레드)는 예외가 나면 포트를 `close()`→`open()`으로 스스로 재연결
하는데, `buf_write()`(쓰기 경로 — 모터 워치독의 강제 0속도 재전송도 결국
이걸 탄다)는 그동안 예외를 잡아 로그만 남기고 포트는 그대로 뒀다. 포트가
일시적 지연이 아니라 실제로 막힌 상태라면, 그 뒤 모든 쓰기(워치독의 재시도
포함)가 계속 같은 이유로 조용히 실패할 수 있었다 — 아래 두 시험은 `buf_
write()`도 `recv_task`와 같은 재연결을 시도하는지, 그리고 재연결 뒤엔
실제로 다시 쓰기가 되는지를 검증한다.
"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

import pytest

_SDK_DIR = (Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "driver"
           / "ros_robot_controller" / "ros_robot_controller")
sys.path.insert(0, str(_SDK_DIR))

import ros_robot_controller_sdk as sdk  # noqa: E402


class _FakePort:
    """serial.Serial 대역 — 실제 장치 없이 Board()를 생성하기 위한 것.

    write()는 넘어온 바이트열을 그냥 쌓기만 한다. read()는 항상 빈 bytes를
    내되, recv_task가 매 사이클 이걸 도는 걸 흉내내려고 짧게 잔다(그래야
    enable_recv=False일 때의 0.01초 sleep 분기와 비슷하게 CPU를 안 먹는다)."""

    def __init__(self, *_a, **kw):
        self.write_timeout = kw.get("write_timeout")
        self.timeout = kw.get("timeout")
        self.writes: list[bytes] = []
        self.rts = None
        self.dtr = None
        self._device = None
        self.opened = False
        # 2026-09-06 — buf_write()의 재연결 시험용. Board.__init__()이 이미
        # open()을 한 번 호출하므로, 재연결이 실제로 일어났는지 보려면
        # "그 뒤로 몇 번 더" 불렸는지가 필요하다.
        self.open_count = 0
        self.close_count = 0

    def setPort(self, device):
        self._device = device

    def open(self):
        self.opened = True
        self.open_count += 1

    def close(self):
        self.opened = False
        self.close_count += 1

    def write(self, buf):
        self.writes.append(bytes(buf))

    def read(self, *_a, **_kw):
        time.sleep(0.01)
        return b""


@pytest.fixture
def fake_port(monkeypatch):
    """serial.Serial(...) 호출을 가로채 _FakePort 인스턴스를 대신 낸다."""
    created: list[_FakePort] = []

    def _factory(*a, **kw):
        port = _FakePort(*a, **kw)
        created.append(port)
        return port

    monkeypatch.setattr(sdk.serial, "Serial", _factory)
    yield created


def test_write_timeout이_pyserial에_실제로_전달된다(fake_port):
    board = sdk.Board(motor_watchdog_timeout=100.0)  # 이 시험에선 워치독이 안 끼어들게 크게 둔다
    assert fake_port[0].write_timeout == sdk.DEFAULT_WRITE_TIMEOUT_S
    assert fake_port[0].write_timeout is not None  # None(무기한 블록)이던 예전 상태로 돌아가면 안 된다


def test_write_timeout값을_직접_줄_수도_있다(fake_port):
    board = sdk.Board(write_timeout=0.05, motor_watchdog_timeout=100.0)
    assert fake_port[0].write_timeout == 0.05


def test_쓰기가_타임아웃돼도_buf_write는_예외를_안_던진다(fake_port, capsys):
    board = sdk.Board(motor_watchdog_timeout=100.0)
    port = fake_port[0]

    def _raise(_buf):
        raise sdk.serial.SerialTimeoutException("write timeout")
    port.write = _raise

    board.set_led(0.1, 0.1, 1, 1)   # 예외가 여기서 새면 이 줄에서 실패한다

    out = capsys.readouterr().out
    assert "유실" in out


def test_모터_명령이_끊기면_워치독이_스스로_0속도를_재전송한다(fake_port):
    board = sdk.Board(motor_watchdog_timeout=0.05, motor_watchdog_poll=0.01)
    port = fake_port[0]

    board.set_motor_speed([[1, 0.3], [2, 0.3], [3, 0.3], [4, 0.3]])
    port.writes.clear()   # 위 정상 명령 자체는 이 시험의 관심사가 아니다

    time.sleep(0.2)   # motor_watchdog_timeout(0.05초)을 넉넉히 넘긴다

    assert port.writes, "워치독이 아무것도 안 보냈다 — 0속도 재전송이 안 걸렸다"
    # 마지막으로 보낸 패킷이 4모터 전부 0속도인지 바이트 단위로 확인한다.
    data = [0x01, 4]
    for motor_id in (1, 2, 3, 4):
        data.extend(struct.pack("<Bf", motor_id - 1, 0.0))
    buf = [0xAA, 0x55, int(sdk.PacketFunction.PACKET_FUNC_MOTOR), len(data)]
    buf.extend(data)
    buf.append(sdk.checksum_crc8(bytes(buf[2:])))
    expected = bytes(buf)
    assert expected in port.writes


def test_계속_새_명령이_오면_워치독이_안_끼어든다(fake_port):
    board = sdk.Board(motor_watchdog_timeout=0.08, motor_watchdog_poll=0.01)
    port = fake_port[0]

    deadline = time.monotonic() + 0.2
    sent = 0
    while time.monotonic() < deadline:
        board.set_motor_speed([[1, 0.1], [2, 0.1], [3, 0.1], [4, 0.1]])
        sent += 1
        time.sleep(0.02)   # watchdog timeout(0.08초)보다 훨씬 촘촘하게

    # 내가 보낸 것 이상으로 워치독이 추가로 끼어들어 쏘지 않았어야 한다 —
    # 끼어들었다면 워치독이 healthy 상태에서도 오발동한다는 뜻이다.
    assert len(port.writes) == sent


def test_쓰기가_계속_실패하면_buf_write가_포트를_재연결한다(fake_port):
    """2026-09-06 — recv_task는 예외가 나면 포트를 스스로 재연결하는데
    buf_write는 그동안 로그만 남기고 포트를 안 건드렸다. "모터 명령은
    가는데 바퀴만 안 움직이고 유일한 해결책이 재기동이었다"는 반복 신고의
    유력 원인이라, buf_write도 같은 재연결(close→open)을 시도하는지
    확인한다."""
    board = sdk.Board(motor_watchdog_timeout=100.0)
    port = fake_port[0]
    port.open_count = 0   # Board.__init__() 안의 최초 open()은 이 시험의 관심사가 아니다

    def _raise(_buf):
        raise sdk.serial.SerialTimeoutException("write timeout")
    port.write = _raise

    board.set_led(0.1, 0.1, 1, 1)   # 실패 -> 재연결을 시도해야 한다

    assert port.close_count == 1, "실패한 쓰기 뒤 포트를 닫지 않았다 — recv_task와 다른 동작"
    assert port.open_count == 1, "닫은 뒤 다시 열지 않았다 — 포트가 막힌 채로 남는다"


def test_재연결_후_다음_쓰기는_다시_성공한다(fake_port):
    """재연결 자체가 다음 명령까지 계속 막아서는 안 된다 — 재연결 이후에
    오는 정상 쓰기(모터 워치독의 다음 재전송 포함)는 다시 성공해야
    한다(2026-09-06)."""
    board = sdk.Board(motor_watchdog_timeout=100.0)
    port = fake_port[0]

    calls = {"n": 0}
    _orig_writes = port.writes

    def _fail_once_then_record(buf):
        calls["n"] += 1
        if calls["n"] == 1:
            raise sdk.serial.SerialTimeoutException("write timeout")
        _orig_writes.append(bytes(buf))
    port.write = _fail_once_then_record

    board.set_led(0.1, 0.1, 1, 1)   # 1번째 — 실패하고 재연결됨
    board.set_led(0.1, 0.1, 1, 1)   # 2번째 — 재연결된 같은 포트로 다시 성공해야 한다

    assert len(port.writes) == 1, "재연결 뒤 정상 쓰기가 기록되지 않았다 — 여전히 막혀 있다"
