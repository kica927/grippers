#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""녹화한 텔레옵 rosbag2 를 LeRobot 데이터셋으로 바꾼다.

    python3 tools/vla/bag_to_lerobot.py /grippers/recordings/demo_20260830_141230 \
        --repo-id kica927/grippers-pick --task "체스말을 집어 바구니에 넣는다"

**먼저 --dry-run 으로 확인할 것.** 시연을 다 찍고 나서야 데이터가 못 쓰는
것이었다는 걸 알면 되돌릴 수 없다. dry-run 은 lerobot 없이도 돌아서 파이
안에서 녹화 직후 바로 확인할 수 있다.

    python3 tools/vla/bag_to_lerobot.py <bag> --dry-run

## 무엇을 읽는가

    /gripper_cam/image_raw/compressed   기준 시계 · observation.images.gripper
    /teleop/follower_present            observation.state  (팔이 실제로 있는 곳)
    /teleop/follower_counts             action             (팔에 내린 목표)
    /teleop/engaged                     에피소드 경계

`--top-camera` 로 탑뷰/뎁스캠 압축 토픽을 하나 더 붙일 수 있다.

## 왜 그리퍼캠이 기준 시계인가

토픽마다 주기가 다르다(카메라 15Hz, 텔레옵 50Hz). 하나에 맞춰야 하는데,
**이미지는 만들어낼 수 없고 명령은 만들어낼 수 있다.** 없는 프레임을
지어내는 것보다 있는 명령을 계단 보간으로 늘리는 쪽이 정직하다.

## 판단은 여기서 하지 않는다

무엇이 한 프레임이 되는가는 전부 `episode_spec.py` 에 있다. 이 파일은
bag 을 읽고 데이터셋을 쓰는 배관일 뿐이다 — 그래야 판단 로직이 ROS 없이
테스트된다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import episode_spec as spec  # noqa: E402

GRIPPER_TOPIC = "/gripper_cam/image_raw/compressed"
STATE_TOPIC = "/teleop/follower_present"
ACTION_TOPIC = "/teleop/follower_counts"
ENGAGED_TOPIC = "/teleop/engaged"

# 30 은 LeRobot 데이터셋 메타의 fps 다. 실제 프레임 간격은 카메라(15Hz)가
# 정하지만, 정책이 학습할 때 쓰는 것은 프레임 순서이지 벽시계가 아니다.
# 그리퍼캠 주기를 바꾸면 여기도 같이 바꿀 것.
FPS_DEFAULT = 15


def _read_bag(path: Path, topics: list, storage_id: str = "sqlite3") -> dict:
    """bag 에서 필요한 토픽만 시각과 함께 읽는다.

    시각은 bag 이 기록한 수신 시각(나노초)을 쓴다. 메시지 안의
    header.stamp 가 아니다 — /teleop/* 중 Int32MultiArray 와 Bool 은
    header 가 없어서 둘을 섞으면 시계가 갈라진다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    missing = [t for t in topics if t not in types]
    if missing:
        raise SystemExit(
            "bag 에 없는 토픽: " + ", ".join(missing) + "\n"
            "  있는 토픽: " + ", ".join(sorted(types)) + "\n"
            "  그리퍼캠이 없다면 record_demo.sh 를 --with-camera 로 찍었는지,\n"
            "  gripper_cam_publisher_node 가 떠 있었는지 확인하세요."
        )

    wanted = set(topics)
    out = {t: {"t": [], "v": []} for t in topics}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        out[topic]["t"].append(stamp_ns / 1e9)
        out[topic]["v"].append(msg)
    return out


def _counts(msgs: list) -> list:
    """Int32MultiArray 목록을 관절 6개 리스트 목록으로."""
    return [list(m.data) for m in msgs]


def _unwrap_per_joint(series: list) -> list:
    """관절별로 따로 편다. 프레임별 리스트 -> 관절별 수열 -> 다시 프레임별."""
    if not series:
        return []
    by_joint = [spec.unwrap_series([f[j] if j < len(f) else None for f in series])
                for j in range(spec.JOINT_COUNT)]
    return [[by_joint[j][i] for j in range(spec.JOINT_COUNT)]
            for i in range(len(series))]


def analyse(bag: Path, top_camera: str | None, storage_id: str = "sqlite3"):
    """bag 을 읽어 프레임 목록과 보고서를 만든다. 데이터셋은 안 쓴다."""
    topics = [GRIPPER_TOPIC, STATE_TOPIC, ACTION_TOPIC, ENGAGED_TOPIC]
    if top_camera:
        topics.append(top_camera)
    data = _read_bag(bag, topics, storage_id)

    ref_t = data[GRIPPER_TOPIC]["t"]
    if not ref_t:
        raise SystemExit("그리퍼캠 프레임이 0개입니다 — 녹화에 영상이 없습니다.")

    # 기준 시계에 맞춰 계단 보간, 그 다음 관절별로 unwrap.
    #
    # 샘플링을 먼저 해도 되는 이유: unwrap 은 이웃한 두 값의 **최단 회전**을
    # 쓰는데, 그것이 진짜 움직임이려면 두 값 사이의 실제 이동이 반 바퀴
    # (2048카운트)보다 작아야 한다. 텔레옵은 한 패킷당 80카운트로 잘리고
    # (follower_teleop_node --slew) 패킷은 50Hz 이므로, 카메라 한 프레임
    # (66ms, 약 3패킷) 사이의 이동은 최대 267카운트다. 여유가 7배 넘는다.
    #
    # 그리퍼캠을 훨씬 느리게 돌리거나 슬루를 크게 올리면 이 전제가 깨진다.
    # 그때는 50Hz 원본 수열을 먼저 펴고 나서 샘플링해야 한다.
    state = _unwrap_per_joint(spec.hold_sample(
        ref_t, data[STATE_TOPIC]["t"], _counts(data[STATE_TOPIC]["v"])))
    action = _unwrap_per_joint(spec.hold_sample(
        ref_t, data[ACTION_TOPIC]["t"], _counts(data[ACTION_TOPIC]["v"])))
    engaged = spec.hold_sample(
        ref_t, data[ENGAGED_TOPIC]["t"], [m.data for m in data[ENGAGED_TOPIC]["v"]])

    report = spec.BuildReport()
    frames = []
    for ep in spec.engaged_episodes(engaged):
        rows, ds, da = spec.build_frames(ep, state, action)
        report.dropped_missing_state += ds
        report.dropped_missing_action += da
        if rows:
            report.episodes.append(ep)
            report.kept += len(rows)
            frames.append((ep, rows))
    return frames, report, data, ref_t


def _decode(msg):
    import cv2
    import numpy as np
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("JPEG 디코드 실패")
    return img[:, :, ::-1]          # BGR -> RGB (LeRobot 규약)


def write_dataset(frames, data, ref_t, args) -> None:
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    image_keys = ["gripper"] + (["top"] if args.top_camera else [])
    sample = _decode(data[GRIPPER_TOPIC]["v"][0])
    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=spec.lerobot_features(image_keys, sample.shape),
        root=args.out,
        use_videos=True,
    )

    top = None
    if args.top_camera:
        top = spec.hold_sample(ref_t, data[args.top_camera]["t"],
                               data[args.top_camera]["v"])

    for ep, rows in frames:
        for row in rows:
            frame = {
                "observation.state": np.array(row.state, dtype=np.float32),
                "action": np.array(row.action, dtype=np.float32),
                "observation.images.gripper": _decode(
                    data[GRIPPER_TOPIC]["v"][row.ref_index]),
            }
            if top is not None:
                if top[row.ref_index] is None:
                    continue
                frame["observation.images.top"] = _decode(top[row.ref_index])
            ds.add_frame(frame, task=args.task)
        ds.save_episode()
        print(f"  에피소드 #{ep.index} 저장 — 프레임 {len(rows)}개")


def main() -> int:
    ap = argparse.ArgumentParser(description="텔레옵 rosbag2 -> LeRobot 데이터셋")
    ap.add_argument("bag", type=Path, help="rosbag2 디렉터리")
    ap.add_argument("--repo-id", default="local/grippers-demo")
    ap.add_argument("--task", default="체스말을 집어 바구니에 넣는다",
                    help="SmolVLA 가 읽는 자연어 지시. 에피소드마다 같은 문장을 쓴다")
    ap.add_argument("--out", type=Path, default=None, help="데이터셋 저장 위치")
    ap.add_argument("--fps", type=int, default=FPS_DEFAULT)
    ap.add_argument("--top-camera", default=None,
                    help="탑뷰/뎁스캠 압축 토픽 (예: /depth_cam/image_raw/compressed)")
    ap.add_argument("--storage", default="sqlite3",
                    help="rosbag2 저장 형식. Humble 기본은 sqlite3, mcap 도 가능")
    ap.add_argument("--dry-run", action="store_true",
                    help="lerobot 없이 무엇이 만들어질지만 본다")
    args = ap.parse_args()

    if not args.bag.exists():
        raise SystemExit(f"없는 경로: {args.bag}")

    frames, report, data, ref_t = analyse(args.bag, args.top_camera, args.storage)
    print(f"기준 프레임(그리퍼캠) {len(ref_t)}개 · {ref_t[-1] - ref_t[0]:.1f}초")
    print(report.summary())

    if not report.episodes:
        raise SystemExit(
            "\n쓸 수 있는 에피소드가 없습니다.\n"
            "  · /teleop/engaged 가 한 번도 True 가 아니었다면 텔레옵에서 `f` 로\n"
            "    팔 추종을 켠 채 시연했는지 확인하세요.\n"
            "  · state 결측으로 전부 버려졌다면 --state-period 0 으로 찍힌\n"
            "    녹화입니다 — 실측 자세 없이는 학습 데이터가 되지 않습니다."
        )
    if args.dry_run:
        print("\n--dry-run: 데이터셋은 쓰지 않았습니다.")
        return 0

    write_dataset(frames, data, ref_t, args)
    print(f"\n완료 — {args.out or '기본 위치'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
