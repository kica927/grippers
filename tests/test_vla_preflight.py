"""실기 앞에서 돌리는 점검 도구 (2026-08-30, VLA 스트레치).

## 왜 이걸 테스트하나

`selftest.py` 는 파이 컨테이너에서 pytest 없이 도는 축소판이다. 축소판이
**썩는 것**이 위험하다 — 맥에서는 93개가 통과하는데 파이에서 도는 11개가
낡은 계약을 보고 있으면, 배포가 잘못된 날에도 초록불이 나온다.

그래서 여기서 selftest 를 실제로 실행한다. 본체가 바뀌면 여기서 먼저
깨진다.
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SELFTEST = ROOT / "tools" / "vla" / "selftest.py"
PREFLIGHT = ROOT / "tools" / "vla" / "preflight.sh"
PLAN = ROOT / "tools" / "vla" / "COLLECTION_PLAN.md"


def _run_selftest():
    return subprocess.run([sys.executable, str(SELFTEST)],
                          capture_output=True, text=True)


def test_자체_점검이_통과한다():
    """파이에서 돌 축소판이 지금 코드로 통과해야 한다."""
    result = _run_selftest()

    assert result.returncode == 0, result.stdout + result.stderr


def test_자체_점검이_실패를_실패로_알린다():
    """전부 통과만 찍는 점검은 점검이 아니다. 일부러 어긋낸 값으로
    실패 경로가 실제로 도는지 본다."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("vla_selftest", SELFTEST)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    module._fails.clear()
    module.check("일부러 틀린 것", 1, 2)

    assert module._fails == ["일부러 틀린 것"]


def test_점검_항목이_충분히_있다():
    """항목이 한둘로 줄면 '배포가 맞는가'를 못 본다."""
    result = _run_selftest()

    assert result.stdout.count(" OK ") >= 10


# ── 읽기 전용이라는 약속 ───────────────────────────────────────────────────


def test_preflight_가_아무것도_바꾸지_않는다():
    """실기 앞에서 도는 도구가 상태를 바꾸면, 점검이 원인이 된다.
    pi_capture 의 preflight 와 같은 약속이다."""
    text = PREFLIGHT.read_text(encoding="utf-8")
    body = "\n".join(line for line in text.splitlines()
                     if not line.strip().startswith("#"))

    # 단어 경계로 본다. 부분 문자열로 찾으면 `/dev/soarm ` 이 `rm ` 에
    # 걸린다 — 그런 테스트는 통과시키려고 코드를 비틀게 만든다.
    for forbidden in [r"\bpkill\b", r"\bkill\b", r"ros2 topic pub",
                      r"ros2 param set", r"ros2 lifecycle", r"colcon build",
                      r"(?<![\w/])rm\s+-", r"(?<![\w/])mv\s"]:
        assert not re.search(forbidden, body), \
            f"preflight 가 {forbidden!r} 에 걸린다 — 읽기 전용이어야 한다"


def test_preflight_가_서보에_쓰지_않는다():
    """복구 도구를 부르되 --apply 없이 읽기만 해야 한다."""
    text = PREFLIGHT.read_text(encoding="utf-8")

    assert "restore_taught_offsets.py" in text
    assert "--apply" not in text


def test_preflight_가_막힘을_종료코드로_알린다():
    """사람이 화면을 잘못 읽는 것보다 종료코드가 정확하다."""
    text = PREFLIGHT.read_text(encoding="utf-8")

    assert "exit 1" in text and "exit 0" in text


# ── 계획 문서 ──────────────────────────────────────────────────────────────


def test_계획에_작업_규칙이_들어_있다():
    """인수인계 문서에는 기술 내용만이 아니라 접속·셸 규칙이 같이 있어야
    한다. 그것이 없어서 막히는 일이 반복됐다."""
    text = PLAN.read_text(encoding="utf-8")

    assert "ssh pi@raspberrypi.local" in text
    assert "exec_shell.sh" in text
    assert "setup.zsh" in text
    assert "ROS_DOMAIN_ID=21" in text


def test_계획이_평가용_구간을_남긴다():
    """전부 학습에 넣으면 외웠는지 배웠는지 구분할 수 없다."""
    text = PLAN.read_text(encoding="utf-8")

    assert "평가" in text and "빼세요" in text


def test_계획이_지시_문장을_고정하라고_말한다():
    """에피소드마다 다르면 정책이 문장과 동작을 잘못 엮는다."""
    text = PLAN.read_text(encoding="utf-8")

    assert "--task" in text
