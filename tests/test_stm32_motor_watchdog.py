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


# ---------------------------------------------------------------------------
# 회전 정지 확인 (2026-09-07 실기 사고 후속) — 모듈 docstring/
# ros_robot_controller_sdk.py 상단 ROTATION_STALL_* 주석 참고.
#
# 2026-09-07 실기: 정지 명령(mission_orchestrator의 base.stop(), 이
# 모듈의 모터 워치독 둘 다)이 몇 분 내내 예외 없이 계속 "성공"했는데도
# 바퀴는 실제로 멈추지 않았다. 이 SDK엔 바퀴 회전을 STM32가 되읽어오는
# 프로토콜이 아예 없어서(모터는 buf_write 전용, parsers 맵에 엔코더/속도
# 리포트가 없다) 그 자체로는 폐루프 확인이 불가능하다 — 대신 이미 오는
# IMU 자이로(진짜 실측)로 "정지 명령은 계속 나가는데 실제로는 여전히
# 돌고 있다"를 감지한다.
# ---------------------------------------------------------------------------

def _now():
    return time.monotonic()


def test_회전정지_판정_자이로가_크면_감지한다():
    now = _now()
    assert sdk.rotation_stall_detected(
        gz=0.30, gz_at=now - 0.1, idle_s=1.5, now=now,
        gz_threshold_rad_s=0.15, stale_after_s=1.0, min_idle_s=1.0)


def test_회전정지_판정_자이로가_작으면_감지_안한다():
    now = _now()
    assert not sdk.rotation_stall_detected(
        gz=0.05, gz_at=now - 0.1, idle_s=1.5, now=now,
        gz_threshold_rad_s=0.15, stale_after_s=1.0, min_idle_s=1.0)


def test_회전정지_판정_idle이_아직_짧으면_보류한다():
    """워치독이 막 발동한 찰나(관성으로 아직 덜 멈췄을 수 있다)는 오탐하지
    않는다."""
    now = _now()
    assert not sdk.rotation_stall_detected(
        gz=0.30, gz_at=now - 0.1, idle_s=0.5, now=now,
        gz_threshold_rad_s=0.15, stale_after_s=1.0, min_idle_s=1.0)


def test_회전정지_판정_자이로_값이_오래됐으면_보류한다():
    """IMU 스트림 자체가 죽은 상태(recv_task가 별도로 재연결을 시도하는
    상황)면 "모른다"를 "괜찮다"로 오판하지 않되, 이 판정이 오탐을 내지도
    않는다 — 자이로 자체가 안 죽었는데 값만 우연히 옛날 것인 경우와
    실제 스트림 정지를 구분할 방법이 이 함수 수준에는 없어서, 안전한
    쪽(경보 안 함)을 택한다."""
    now = _now()
    assert not sdk.rotation_stall_detected(
        gz=0.30, gz_at=now - 5.0, idle_s=1.5, now=now,
        gz_threshold_rad_s=0.15, stale_after_s=1.0, min_idle_s=1.0)


def test_모터_쓰기가_실패하면_워치독_타임스탬프를_안_갱신한다(fake_port):
    """2026-09-07 2차 회전정지 사고 코드 리뷰 후속 — set_motor_speed()가
    buf_write() 성공 여부와 무관하게 _last_motor_cmd_at을 무조건 찍고
    있었다. 그러면 쓰기가 실제로 실패해도 워치독은 "방금 명령이 나갔다"고
    오판해 idle_s가 안 쌓이고, 다음 판정 주기까지 재시도조차 안 걸린다 —
    실제 사고 원인이라는 증거는 없었지만(로그에 쓰기 예외 자체가 없었다),
    워치독이 "성공"을 잘못 정의하고 있던 것 자체는 결함이다. 쓰기가
    성공했을 때만 타임스탬프를 갱신해야 한다."""
    board = sdk.Board(motor_watchdog_timeout=100.0)
    port = fake_port[0]

    before = board._last_motor_cmd_at

    def _raise(_buf):
        raise sdk.serial.SerialTimeoutException("write timeout")
    port.write = _raise

    board.set_motor_speed([[1, 0.3], [2, 0.3], [3, 0.3], [4, 0.3]])

    assert board._last_motor_cmd_at == before, (
        "쓰기가 실패했는데도 타임스탬프가 갱신됐다 — 워치독이 이 실패를 못 본다")


def test_모터_쓰기가_성공하면_워치독_타임스탬프를_갱신한다(fake_port):
    """위 시험의 반대쪽 — 정상 동작(쓰기 성공)까지 갱신을 막아버리면 안
    된다는 걸 같이 고정해 둔다."""
    board = sdk.Board(motor_watchdog_timeout=100.0)

    before = board._last_motor_cmd_at
    time.sleep(0.01)
    board.set_motor_speed([[1, 0.3], [2, 0.3], [3, 0.3], [4, 0.3]])

    assert board._last_motor_cmd_at > before


def test_회전정지_판정_자이로_값이_아직_없으면_보류한다():
    now = _now()
    assert not sdk.rotation_stall_detected(
        gz=None, gz_at=None, idle_s=1.5, now=now,
        gz_threshold_rad_s=0.15, stale_after_s=1.0, min_idle_s=1.0)


def test_imu_수신시_엿보기_캐시가_갱신된다(fake_port):
    """packet_report_imu가 소비형 큐(get_imu())와 별개로, 워치독이 읽는
    엿보기 캐시(_last_imu_gz/_last_imu_at)도 갱신하는지 확인한다 — 두
    소비자가 같은 큐를 다투면 데이터를 나눠 갖게 되므로 별도 캐시가
    필요하다(파일 상단 주석 참고)."""
    board = sdk.Board(motor_watchdog_timeout=100.0)
    assert board._last_imu_gz is None

    gz_value = 0.42
    data = struct.pack('<6f', 0.0, 0.0, 9.8, 0.0, 0.0, gz_value)
    board.packet_report_imu(data)

    assert board._last_imu_gz == pytest.approx(gz_value)
    assert board._last_imu_at is not None

    # get_imu()로 큐를 소비해도 엿보기 캐시는 그대로 남아 있어야 한다 —
    # 워치독과 pub_imu_data가 서로 안 다퉈야 한다는 게 이 캐시의 요점이다.
    board.get_imu()
    assert board._last_imu_gz == pytest.approx(gz_value)


def test_회전중_워치독_재전송에도_계속_돌면_경보하고_멈추면_그친다(fake_port, capsys, monkeypatch):
    """통합 시험 — 실제로 모터 워치독 스레드가 자이로 값을 보고 경보
    문구를 찍는지, 그리고 자이로가 실제로 잠잠해지면 경보가 그치는지
    확인한다. 1초 단위 실기 튜닝값(ROTATION_STALL_MIN_IDLE_S 등)을 실제로
    기다리면 시험이 느려지므로, 모듈 전역을 짧게 monkeypatch한다 —
    _motor_watchdog_task가 이 전역을 매번 다시 읽어서 넘기도록 짜여 있어
    반영된다(ros_robot_controller_sdk.py의 rotation_stall_detected
    호출부 주석 참고).

    ⚠️ 두 상황(계속 돎 / 멈춤)을 별개 테스트 대신 **같은 board 하나로
    이어서** 확인한다 — Board()에는 워치독 데몬 스레드를 깨끗이 멈추는
    방법이 없어서, 테스트마다 새 Board()를 만들면 이전 테스트의 워치독
    스레드가 안 죽고 계속 돌며 다음 테스트의 capsys 캡처 구간에 자기
    출력을 섞어 넣는다(처음엔 이 파일도 그렇게 짰다가, "멈추면 경보 안
    함" 테스트가 직전 테스트의 board가 낸 경보 잔재 때문에 거짓 실패하는
    걸 보고 이 형태로 고쳤다) — 같은 board를 재사용하면 그 board의
    이력만 보게 되고, readouterr()로 버퍼를 매번 비우면 이전 구간의
    출력이 다음 assert에 안 섞인다."""
    monkeypatch.setattr(sdk, "ROTATION_STALL_MIN_IDLE_S", 0.03)
    monkeypatch.setattr(sdk, "ROTATION_STALL_IMU_STALE_S", 1.0)
    monkeypatch.setattr(sdk, "ROTATION_STALL_GZ_THRESHOLD_RAD_S", 0.15)

    board = sdk.Board(motor_watchdog_timeout=0.02, motor_watchdog_poll=0.01)

    # 1) 자이로가 계속 큰 값을 보고하는 상황을 흉내낸다 — 정지 명령
    # (워치독의 반복 재전송)에도 불구하고 실측 회전이 여전히 크다.
    data = struct.pack('<6f', 0.0, 0.0, 9.8, 0.0, 0.0, 0.40)
    board.packet_report_imu(data)
    time.sleep(0.25)   # min_idle_s(0.03) + 워치독이 몇 사이클 더 돌 여유

    out = capsys.readouterr().out
    assert "경고" in out and "자이로" in out, (
        f"자이로가 계속 크게 나오는데도 경보가 안 찍혔다 — 회귀. 출력: {out!r}")

    # 2) 이제 자이로가 실제로 정지 상태 노이즈 수준으로 잠잠해진다.
    data = struct.pack('<6f', 0.0, 0.0, 9.8, 0.0, 0.0, 0.02)
    board.packet_report_imu(data)
    time.sleep(0.25)

    out = capsys.readouterr().out
    assert "경고" not in out, f"실제로 멈췄는데 경보가 찍혔다 — 오탐. 출력: {out!r}"
