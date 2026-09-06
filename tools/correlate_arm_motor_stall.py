#!/usr/bin/env python3
"""그리퍼/팔 버스 정지가 바퀴 모터 워치독을 유발하는지 확증한다 (2026-09-06).

## 무엇을 확증하려는가

"모터가 안 돌아간다"는 실기 보고의 유력 원인으로 다음 인과 사슬을 세웠다:

    그리퍼/팔 버스 write()가 걸린다(third_party/soarm_provided_d/soarm_lab/
    driver_sdk.py, write_timeout 결함)
        -> CARRY 상태의 get_load() 호출이 늦어진다
        -> 그 사이클의 apply_velocity() 재전송이 늦어진다
        -> STM32 바퀴 모터 워치독(0.5초, ros_robot_controller_sdk.py)이
           대신 걸려 0속도를 강제 전송한다

이건 코드를 읽고 세운 가설이지 실기로 확인한 사실이 아니다. 이 스크립트는
실기에서 한 번 돌린 로그만으로 "정말 그런가"를 수치로 확인한다 — 감으로
"둘 다 로그에 있으니 맞겠지"가 아니라, **모터 워치독이 보고하는 idle_s
(마지막 모터 명령 이후 경과 시간)를 거꾸로 풀어 "이 정지가 언제
시작됐는가"를 계산하고, 그 구간 안에 WRITE_TIMEOUT이 실제로 있었는지를
맞춰 본다.**

## 전제 조건

두 로그 줄 모두 이 스크립트를 만든 날(2026-09-06) `time.time()`(벽시계
epoch, 초) 타임스탬프를 앞에 붙이도록 고쳤다 —

    [1725600000.123] [motor_watchdog] 0.62초 동안 새 모터 명령이 없습니다 — ...
    [1725599999.510] [driver] WRITE_TIMEOUT servo=6 — ...

ros_robot_controller_node(STM32/바퀴)와 arm_driver_node(SO-ARM101/그리퍼)는
서로 다른 ROS2 노드·프로세스지만 **같은 Pi 한 대**에서 돈다 — 그래서 같은
`time.time()` 기준이면 파일 안에서 줄이 어떤 순서로 섞여 있든(비동기
stdout이라 완벽한 시간순 보장은 없다) 상관없이 정확히 비교할 수 있다.
Host(맥)의 시계와는 비교하지 않는다 — 두 기기 시계는 어긋날 수 있어서,
이 확증은 **Pi 로그 한 파일 안에서만** 끝낸다.

## 쓰는 법

    Pi에서 `ros2 launch ... > /tmp/bringup.log 2>&1`로 미션을 한 번 돌린 뒤
    (bringup_now.sh가 이 형태로 띄운다 — grippers-*-bringup-ops-scripts
    메모 참고), 그 로그 파일을 이 스크립트로 분석한다.

        python3 tools/correlate_arm_motor_stall.py /tmp/bringup.log
        python3 tools/correlate_arm_motor_stall.py /tmp/bringup.log --window 0.3

    `--window`(기본 0.2초)는 print()가 실제로 그 순간 flush되기까지의
    잡음을 흡수하는 여유값이다 — 너무 좁히면 진짜 원인도 놓치고, 너무
    넓히면 우연의 일치까지 "확증됨"으로 센다.

## 결과를 어떻게 읽는가

    matched(모터 워치독 중 그 idle 구간에 WRITE_TIMEOUT이 있었던 것) 비율이
    높을수록 가설을 뒷받침한다. 0%면 가설이 틀렸다는 뜻이다 — 그 경우
    "모터가 안 돌아간다" 증상의 원인은 다른 곳(예: FSM 자체의 다른 블로킹
    호출, 별개의 배터리/전원 문제)에 있다는 뜻이니 그쪽을 다시 봐야 한다.
    unmatched 모터 워치독이 있다고 가설이 완전히 틀린 것은 아니다 — 이
    결함 말고 다른 원인으로도 워치독이 걸릴 수 있다(그래서 unmatched
    목록을 같이 보여준다 — 그 시각대에 다른 무슨 일이 있었는지는 사람이
    bringup.log를 직접 봐야 한다).
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass

MOTOR_WATCHDOG_RE = re.compile(
    r"\[(?P<ts>\d+\.\d+)\]\s*\[motor_watchdog\]\s*(?P<idle>\d+\.\d+)초")
WRITE_TIMEOUT_RE = re.compile(
    r"\[(?P<ts>\d+\.\d+)\]\s*\[driver\]\s*WRITE_TIMEOUT")

DEFAULT_WINDOW_S = 0.2


@dataclass
class MotorStallEvent:
    ts: float          # 워치독이 강제 0속도를 보낸 시각
    idle_s: float      # 그때까지 새 모터 명령이 없었던 시간
    stall_start: float  # 역산한 정지 시작 시각 (ts - idle_s)


def _parse(path: str) -> tuple[list[MotorStallEvent], list[float]]:
    motor_events: list[MotorStallEvent] = []
    write_timeouts: list[float] = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = MOTOR_WATCHDOG_RE.search(line)
            if m:
                ts = float(m.group("ts"))
                idle_s = float(m.group("idle"))
                motor_events.append(MotorStallEvent(ts, idle_s, ts - idle_s))
                continue
            w = WRITE_TIMEOUT_RE.search(line)
            if w:
                write_timeouts.append(float(w.group("ts")))
    return motor_events, write_timeouts


def correlate(motor_events: list[MotorStallEvent], write_timeouts: list[float],
             window_s: float = DEFAULT_WINDOW_S):
    """모터 워치독 각각에 대해, 역산한 정지 구간
    [stall_start - window_s, ts + window_s] 안에 WRITE_TIMEOUT이 있는지 본다.

    반환: (matched, unmatched, orphan_write_timeouts)
      matched   — [(motor_event, 대응 write_timeout_ts), ...]
      unmatched — [motor_event, ...] (대응하는 write_timeout을 못 찾음)
      orphans   — 어떤 모터 워치독과도 안 엮인 write_timeout 시각들
    """
    matched = []
    unmatched = []
    used_write_timeouts: set[int] = set()  # write_timeouts 안의 index

    for event in motor_events:
        lo = event.stall_start - window_s
        hi = event.ts + window_s
        found_idx = None
        for idx, wt in enumerate(write_timeouts):
            if idx in used_write_timeouts:
                continue
            if lo <= wt <= hi:
                found_idx = idx
                break
        if found_idx is not None:
            used_write_timeouts.add(found_idx)
            matched.append((event, write_timeouts[found_idx]))
        else:
            unmatched.append(event)

    orphans = [wt for idx, wt in enumerate(write_timeouts)
              if idx not in used_write_timeouts]
    return matched, unmatched, orphans


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log_path", nargs="?", default="/tmp/bringup.log",
                        help="Pi의 bringup 로그 경로 (기본 /tmp/bringup.log)")
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                        help=f"대조 여유 시간(초), 기본 {DEFAULT_WINDOW_S}")
    args = parser.parse_args()

    try:
        motor_events, write_timeouts = _parse(args.log_path)
    except FileNotFoundError:
        print(f"로그 파일을 찾을 수 없다: {args.log_path}", file=sys.stderr)
        return 1

    print(f"파일: {args.log_path}")
    print(f"모터 워치독 발동: {len(motor_events)}건")
    print(f"그리퍼/팔 버스 WRITE_TIMEOUT: {len(write_timeouts)}건")
    print()

    if not motor_events:
        print("모터 워치독이 이 로그에 한 번도 없다 — 이 실행에서는 애초에 "
             "'모터가 안 돌아간다' 증상 자체가 안 나타났다는 뜻이다. "
             "증상이 있었던 로그로 다시 돌릴 것.")
        return 0

    matched, unmatched, orphans = correlate(motor_events, write_timeouts, args.window)
    ratio = len(matched) / len(motor_events) * 100

    print(f"확증됨(모터 워치독 직전 정지 구간에 WRITE_TIMEOUT 있음): "
         f"{len(matched)}/{len(motor_events)}건 ({ratio:.0f}%)")
    for event, wt in matched:
        gap = event.ts - wt
        print(f"  [모터 {event.ts:.3f}, idle={event.idle_s:.2f}s] "
             f"<- WRITE_TIMEOUT {wt:.3f} (모터 발동 {gap:.2f}초 전)")

    if unmatched:
        print()
        print(f"대응하는 WRITE_TIMEOUT을 못 찾은 모터 워치독: {len(unmatched)}건 "
             "(다른 원인일 수 있다 — 그 시각대의 bringup.log를 직접 확인할 것)")
        for event in unmatched:
            print(f"  [모터 {event.ts:.3f}, idle={event.idle_s:.2f}s, "
                 f"정지 시작 추정 {event.stall_start:.3f}]")

    if orphans:
        print()
        print(f"모터 워치독으로 안 이어진 WRITE_TIMEOUT: {len(orphans)}건 "
             "(정상 — 짧게 걸렸다 풀려서 그 사이클이 워치독 문턱을 안 넘긴 "
             "경우일 수 있다)")
        for wt in orphans:
            print(f"  [{wt:.3f}]")

    print()
    if ratio >= 50.0:
        print("결론: 가설(그리퍼/팔 버스 정지 -> 바퀴 모터 워치독)이 이 로그로 "
             "뒷받침된다.")
    elif matched:
        print("결론: 일부는 맞지만(matched > 0) 전부는 아니다 — 다른 원인도 "
             "같이 있을 가능성이 있다. unmatched 구간을 사람이 직접 봐야 한다.")
    else:
        print("결론: 이 로그에서는 가설이 뒷받침되지 않는다 — 모터 워치독의 "
             "원인이 그리퍼/팔 버스가 아니라 다른 곳(FSM의 다른 블로킹 호출, "
             "전원 등)일 가능성이 높다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
