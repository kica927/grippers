"""SO-ARM101 벤더 시리얼 드라이버의 write_timeout (2026-09-06).

## 무엇을 고정하는가

third_party/soarm_provided_d/soarm_lab/driver_sdk.py의 `STS3215Driver`가
write_timeout 없이(기본값 None = 쓰기 무기한 블록) `serial.Serial()`을 열고
있었다 — 어제(2026-09-05) STM32 베이스 보드(ros_robot_controller_sdk.py)에서
찾아 고친 것과 정확히 같은 결함이다.

`_transact()`/`ping()`이 이 클래스의 유일한 락(`self._lock`)을 쥔 채
`self.serial.write()`를 부른다 — write()가 한 번이라도 안 끝나고 걸리면
그 락은 다시는 안 풀리고, 그리퍼든 팔 관절이든 이 버스에 있는 모든 서보
읽기·쓰기가 그 순간부터 영원히 막힌다. Pi FSM의 CARRY 상태는 매 사이클
`get_load()`로 그리퍼 부하를 읽는데, 이 정지가 걸리면 그 사이클의 후속
`apply_velocity()` 호출까지 늦어져 STM32 쪽 바퀴 모터 워치독(0.5초)이
대신 걸린다 — "그리퍼 통신이 멈췄는데 증상은 바퀴가 안 도는 것"으로
나타난 이유다(grippers.md Phase 11, ArmLinkWatchdog 참고).

여기서는 `serial.Serial()` 호출에 `write_timeout`이 실제로 전달되는지만
고정한다. 값 자체(DEFAULT_WRITE_TIMEOUT_S=0.1)는 실측이 아니다 — 실기에서
정상 왕복 지연을 재고 조정할 것."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOARM_LAB = ROOT / "third_party" / "soarm_provided_d" / "soarm_lab"


def _load_driver_sdk():
    """test_floor_grasp_profiles.py `_fk()`와 같은 방식 — soarm_lab/
    __init__.py가 패키지 전체를 끌어오므로, 그 디렉터리를 sys.path에 얹어
    `driver_sdk`를 flat 모듈로 직접 import한다."""
    if not SOARM_LAB.exists():
        import pytest
        pytest.skip("third_party/soarm_provided_d 서브모듈이 초기화되지 않았다 "
                    "(git submodule update --init third_party/soarm_provided_d)")
    if str(SOARM_LAB) not in sys.path:
        sys.path.insert(0, str(SOARM_LAB))
    import driver_sdk
    return driver_sdk


class _FakeSerial:
    """`connect()`가 여는 시리얼 포트를 흉내낸다 — 실제 통신은 하지 않는다."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.is_open = True

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass


def test_connect가_write_timeout을_serial에_넘긴다(monkeypatch):
    driver_sdk = _load_driver_sdk()
    calls = []

    def fake_serial(**kwargs):
        calls.append(kwargs)
        return _FakeSerial(**kwargs)

    monkeypatch.setattr(driver_sdk.serial, "Serial", fake_serial)
    monkeypatch.setattr(driver_sdk.time, "sleep", lambda *_: None)

    driver = driver_sdk.STS3215Driver(port="/dev/fake")
    ok = driver.connect()

    assert ok is True
    assert len(calls) == 1
    assert "write_timeout" in calls[0], (
        "serial.Serial()에 write_timeout이 없다 — 쓰기가 무기한 블록될 수 있다"
        "(이 결함이 바로 2026-09-06에 찾은 것이다)")
    assert calls[0]["write_timeout"] is not None
    assert calls[0]["write_timeout"] > 0.0


def test_write_timeout을_생성자에서_바꿀_수_있다(monkeypatch):
    """실기 왕복 지연 실측 후 값을 조정할 수 있어야 한다 — STM32 쪽
    Board.__init__(write_timeout=...)와 같은 계약."""
    driver_sdk = _load_driver_sdk()
    calls = []

    def fake_serial(**kwargs):
        calls.append(kwargs)
        return _FakeSerial(**kwargs)

    monkeypatch.setattr(driver_sdk.serial, "Serial", fake_serial)
    monkeypatch.setattr(driver_sdk.time, "sleep", lambda *_: None)

    driver = driver_sdk.STS3215Driver(port="/dev/fake", write_timeout=0.25)
    driver.connect()

    assert calls[0]["write_timeout"] == 0.25


def test_write_타임아웃이_나면_예외가_아니라_None을_돌려준다(monkeypatch):
    """`_transact()`가 SerialTimeoutException을 삼키고 락을 풀어야, 다음
    호출이 재시도할 수 있다 — 여기서 안 삼키면 여전히 무기한 블록과 같은
    결과(콜백 스레드가 예외로 죽어 그 뒤로 응답이 없다)가 난다."""
    driver_sdk = _load_driver_sdk()

    class _TimeoutSerial(_FakeSerial):
        def write(self, _buf):
            raise driver_sdk.serial.SerialTimeoutException("write timeout")

        def flush(self):
            pass

    monkeypatch.setattr(driver_sdk.serial, "Serial",
                        lambda **kw: _TimeoutSerial(**kw))
    monkeypatch.setattr(driver_sdk.time, "sleep", lambda *_: None)

    driver = driver_sdk.STS3215Driver(port="/dev/fake")
    driver.connect()

    result = driver._transact(1, driver_sdk.INST_READ, [0, 1], response_len=1)

    assert result is None
    # 락이 실제로 풀렸는지까지 확인한다 — 여기서 걸려 있으면 다음 줄에서
    # 이 테스트 자체가 멈춘다(타임아웃으로 실패가 드러난다).
    assert driver._lock.acquire(timeout=1.0)
    driver._lock.release()
