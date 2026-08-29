#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VLA 순수 로직 자체 점검 — pytest 없이, 컨테이너 안에서 돈다.

## 왜 따로 있는가

맥에는 `tests/test_vla_*.py` 93개가 있지만 파이 컨테이너에는 pytest 가
없을 수 있고, 있어도 실기 앞에서 전체 스위트를 돌리는 것은 과하다.

여기서 확인하는 것은 딱 하나 — **배포된 코드가 맥에서 검증한 그 코드인가.**
파일을 옮기다 반쪽만 올라가거나, 컨테이너가 옛 사본을 들고 있거나, 파이썬
버전이 달라 동작이 갈리는 일이 실제로 있었다.

실패하면 그 자리에서 멈춘다. 수집은 되돌릴 수 없는 시간이라, 의심스러운
코드로 시작하지 않는다.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import action_chunk as ac  # noqa: E402
import episode_spec as spec  # noqa: E402

_fails: list = []


def check(name: str, got, want) -> None:
    if got == want:
        print(f"  \033[32m OK \033[0m {name}")
    else:
        print(f"  \033[31m XX \033[0m {name}\n         기대 {want}\n         실제 {got}")
        _fails.append(name)


def main() -> int:
    print("=== VLA 순수 로직 자체 점검 ===")

    # 이음매를 움직임으로 오해하지 않는가
    out = spec.unwrap_series([4090, 4095, 5, 10])
    check("카운트 한 바퀴를 편다", [b - a for a, b in zip(out, out[1:])], [5, 6, 5])

    # 결측을 지어내지 않는가
    check("결측은 자리를 지킨다",
          spec.unwrap_series([100, None, 120]), [100, None, 120])

    # 추종 구간만 자르는가
    eps = spec.engaged_episodes([False] * 5 + [True] * 40, settle_frames=3,
                                min_frames=15)
    check("추종 구간만 · latch 직후는 버린다",
          [(e.start, e.end) for e in eps], [(8, 45)])

    # 계단 보간인가
    check("직전 값을 유지한다",
          spec.hold_sample([0.0, 0.1, 0.2], [0.05], ["a"]), [None, "a", "a"])

    # 프레임이 결측을 메우지 않는가
    rows, dropped, _ = spec.build_frames(
        spec.Episode(0, 2), [[1] * 6, [None] * 6], [[2] * 6, [2] * 6])
    check("결측 프레임을 버린다", (len(rows), dropped), (1, 1))

    # LeRobot 이 찾는 이름인가
    feats = spec.lerobot_features(["gripper"])
    check("피처 이름", sorted(feats),
          ["action", "observation.images.gripper", "observation.state"])

    # 액션 속도를 송신 속도와 섞지 않는가
    p = ac.ChunkPlayer(action_hz=15.0, send_hz=50.0)
    check("한 액션을 3틱 보낸다", p.ticks_per_action, 3)

    # 이음매 너머 목표를 최단 회전으로 보는가
    p = ac.ChunkPlayer(slew=80, action_hz=50.0, send_hz=50.0)
    p.prime([5] * 6)
    p.submit([[4100] * 6], now=0.0)
    check("이음매 너머를 최단 회전으로", p.tick(0.0).counts[0], 4)

    # 슬루를 거는가
    p = ac.ChunkPlayer(slew=80, action_hz=50.0, send_hz=50.0)
    p.prime([2048] * 6)
    p.submit([[3000] * 6], now=0.0)
    tick = p.tick(0.0)
    check("슬루로 자르고 보고한다", (tick.counts[0], tick.clamped), (2128, 6))

    # 청크가 떨어져도 데드맨을 굶기지 않는가
    p = ac.ChunkPlayer(action_hz=50.0, send_hz=50.0)
    p.prime([2048] * 6)
    p.submit([[2058] * 6], now=0.0)
    p.tick(0.0)
    check("청크가 떨어져도 계속 보낸다", p.tick(0.02).engaged, True)

    # 데이터셋 fps 와 액션 속도가 같은가
    try:
        import bag_to_lerobot
        check("액션 속도 = 데이터셋 fps",
              ac.ACTION_HZ_DEFAULT, float(bag_to_lerobot.FPS_DEFAULT))
    except Exception as exc:                       # noqa: BLE001
        print(f"  \033[33m ?? \033[0m bag_to_lerobot 를 못 읽었습니다 ({exc})")

    print()
    if _fails:
        print(f"\033[31m{len(_fails)}건 실패 — 배포된 코드가 검증한 것과 다릅니다.\033[0m")
        print("맥에서 다시 배포하고, 컨테이너의 옛 사본이 남아 있지 않은지 보세요.")
        return 1
    print("\033[32m전부 통과 — 배포된 코드가 맥에서 검증한 것과 같습니다.\033[0m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
