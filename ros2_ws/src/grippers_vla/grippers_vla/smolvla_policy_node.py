# -*- coding: utf-8 -*-
"""SmolVLA 정책을 팔로워 텔레옵 수신기에 물린다 (2026-08-30, 스트레치).

## 한 줄 요약

**리더 암이 있던 자리에 정책을 넣는다.** 팔을 움직이는 경로는 바뀌지 않는다.

    사람이 리더 암을 잡음  ──UDP 47800──>  follower_teleop_node ──> 서보
    SmolVLA 정책          ──UDP 47800──>  follower_teleop_node ──> 서보
                              (같은 규약, 같은 포트, 같은 안전장치)

## 왜 새 구동 경로를 만들지 않았는가

팔로워 수신기에는 실기로 다듬은 안전장치가 이미 들어 있다 — 관절 한계
클램프, 한 패킷당 슬루 80카운트, 델타 추종(켜는 순간 팔이 안 튄다),
0.4초 데드맨, 신호가 끊겨도 토크는 유지(들고 있던 물건을 안 떨어뜨림).

정책용으로 서보를 직접 여는 경로를 새로 만들면 저 목록을 전부 다시 구현
하거나 잃는다. 게다가 /dev/soarm 은 한 프로세스만 열 수 있어서, 새로 열면
텔레옵과 동시에 못 쓴다 — 사람이 손으로 이어받는 것이 이 시연의 안전
장치인데 그것을 없애는 셈이다.

## 절대 위치인데 델타 추종에 실려도 되는가

팔로워는 **켜는 순간의 리더/팔로워 자세를 각각 기준점으로 잡고 그 뒤로는
리더의 변화량만** 더한다. 그래서 켤 때 보내는 첫 패킷에 **팔로워의 현재
실측 자세**를 실으면

    leader_ref = follower_ref  =>  target = 보낸 값

이 되어 절대 추종과 같아진다. `_engage()` 가 하는 일이 이것이고, 실측
자세를 못 읽으면 켜지 않는다.

## 추론은 느리고 송신은 빨라야 한다

Pi 5 CPU 에서 한 번 추론이 수백 ms~수 초다. 그동안 패킷을 안 보내면
0.4초 데드맨이 걸린다. 그래서 스레드를 나눈다.

    송신 스레드   50Hz 고정. ChunkPlayer 에서 다음 액션을 꺼내 보낸다
    추론 스레드   되는 대로. 새 청크가 나오면 ChunkPlayer 에 갈아 끼운다

둘 사이를 잇는 규칙(청크 소진 시 붙들기, 낡은 청크 폐기, 슬루)은 전부
`tools/vla/action_chunk.py` 에 순수 함수로 있고 하드웨어 없이 테스트된다.

## 처음 띄울 때는 반드시 --dry-run

`--dry-run` 은 관측·추론·청크 재생을 전부 하되 **engaged=False 로만**
보낸다. 팔은 움직이지 않고, 정책이 무엇을 내려 했는지는 로그로 볼 수
있다. 첫 실행에서 이것 없이 켜지 말 것.
"""

import sys
import threading
import time

sys.path.insert(0, "/grippers/tools/teleop")
sys.path.insert(0, "/grippers/tools/vla")

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import Int32MultiArray  # noqa: E402

import action_chunk  # noqa: E402
import episode_spec  # noqa: E402
from teleop_protocol import DEFAULT_PORT, encode  # noqa: E402

IMAGE_TOPIC_DEFAULT = "gripper_cam/image_raw/compressed"
STATE_TOPIC_DEFAULT = "teleop/follower_present"

# 팔로워와 같은 50Hz. 이보다 느리면 데드맨(0.4초)에 가까워지고, 빠르게
# 보낸다고 팔이 더 부드러워지지는 않는다 — 서보 쓰기가 병목이다.
SEND_HZ_DEFAULT = 50.0

# 관측이 이보다 오래되면 추론하지 않는다. 카메라가 15Hz 이므로 0.5초는
# 7프레임 넘게 빠졌다는 뜻이고, 그건 카메라가 멈춘 것이다.
OBS_STALE_S = 0.5


class SmolvlaPolicyNode(Node):
    def __init__(self):
        super().__init__("smolvla_policy_node")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("task", "체스말을 집어 바구니에 넣는다")
        self.declare_parameter("image_topic", IMAGE_TOPIC_DEFAULT)
        self.declare_parameter("state_topic", STATE_TOPIC_DEFAULT)
        self.declare_parameter("follower_host", "127.0.0.1")
        self.declare_parameter("follower_port", DEFAULT_PORT)
        self.declare_parameter("send_hz", SEND_HZ_DEFAULT)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("dry_run", True)

        self._task = self.get_parameter("task").value
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._send_hz = float(self.get_parameter("send_hz").value)

        self._lock = threading.Lock()
        self._image = None          # 최신 RGB 프레임
        self._image_at = 0.0
        self._state = None          # 최신 실측 자세 (관절 6개)
        self._state_at = 0.0

        self._player = action_chunk.ChunkPlayer()
        self._engaged = False
        self._epoch = 0
        self._seq = 0
        self._stop = threading.Event()

        self.create_subscription(
            CompressedImage, self.get_parameter("image_topic").value,
            self._on_image, 1)
        self.create_subscription(
            Int32MultiArray, self.get_parameter("state_topic").value,
            self._on_state, 1)

        import socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (self.get_parameter("follower_host").value,
                      int(self.get_parameter("follower_port").value))

        self._policy = None
        self._pre = None
        self._post = None
        self._sender = threading.Thread(target=self._send_loop, daemon=True)
        self._infer = threading.Thread(target=self._infer_loop, daemon=True)

        if self._dry_run:
            self.get_logger().warn(
                "--dry-run: 계산만 하고 engaged=False 로 보냅니다 — 팔은 안 움직입니다")
        self.get_logger().info(f"정책 노드 준비 — 팔로워 {self._addr}")

    # ── 관측 ────────────────────────────────────────────────────────────────

    def _on_image(self, msg):
        import cv2
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("JPEG 디코드 실패")
            return
        with self._lock:
            # LeRobot 규약은 RGB 다. 학습 때 bag_to_lerobot 이 같은 변환을
            # 했으므로 여기서 어긋나면 정책이 색이 뒤집힌 세상을 본다.
            self._image = img[:, :, ::-1]
            self._image_at = time.monotonic()

    def _on_state(self, msg):
        data = list(msg.data)
        if len(data) != episode_spec.JOINT_COUNT:
            return
        if any(v == episode_spec.MISSING for v in data):
            return          # 결측이 있는 자세로는 기준점을 못 잡는다
        with self._lock:
            self._state = data
            self._state_at = time.monotonic()

    def _observation(self):
        """추론에 쓸 관측 한 벌. 낡았으면 None."""
        now = time.monotonic()
        with self._lock:
            if self._image is None or self._state is None:
                return None
            if now - self._image_at > OBS_STALE_S:
                return None
            return self._image.copy(), list(self._state)

    # ── 추론 ────────────────────────────────────────────────────────────────

    def _load_policy(self):
        """정책과 **전·후처리기를 같이** 연다.

        ⚠️ LeRobot 0.4.x 는 정규화가 정책 밖으로 빠져 있다. 정책만 열고
        predict_action_chunk 를 부르면

          · 관측: 원시 카운트(약 2048)가 정규화 없이 그대로 들어간다.
            정책은 학습 때 본 적 없는 크기의 숫자를 본다.
          · 액션: 정규화된 값이 그대로 나온다. 그걸 서보 목표로 보내면
            팔이 엉뚱한 곳으로 간다.

        둘 다 예외 없이 조용히 틀린다. 그래서 make_pre_post_processors 로
        체크포인트에 같이 저장된 파이프라인을 반드시 같이 연다
        (lerobot/async_inference/policy_server.py 가 하는 것과 같은 순서)."""
        path = self.get_parameter("policy_path").value
        if not path:
            raise RuntimeError(
                "policy_path 가 비어 있습니다 — 학습한 SmolVLA 체크포인트 경로를 주세요.\n"
                "  아직 학습 전이라면 tools/vla/README.md 의 수집->변환->학습 순서를 보세요.")
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        device = self.get_parameter("device").value
        policy = SmolVLAPolicy.from_pretrained(path)
        policy.to(device)
        policy.eval()
        override = {"device_processor": {"device": device}}
        pre, post = make_pre_post_processors(
            policy.config, pretrained_path=path,
            preprocessor_overrides=override, postprocessor_overrides=override)
        self.get_logger().info(f"정책·전후처리기 적재 완료: {path} ({device})")
        return policy, pre, post

    def _infer_loop(self):
        try:
            self._policy, self._pre, self._post = self._load_policy()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f"정책을 못 띄웁니다 — {exc}")
            self._stop.set()
            return

        import torch
        from lerobot.policies.utils import prepare_observation_for_inference

        device = torch.device(self.get_parameter("device").value)
        while not self._stop.is_set():
            obs = self._observation()
            if obs is None:
                time.sleep(0.05)
                continue
            image, state = obs
            # 이미지는 HWC uint8 로 넘긴다 — prepare_observation_for_inference 가
            # /255, CHW 변환, 배치 차원, 디바이스 이동, task/robot_type 삽입을
            # 한다. ascontiguousarray 가 필요한 이유: BGR->RGB 를 [:, :, ::-1]
            # 로 했으므로 stride 가 음수이고, torch.from_numpy 는 음수 stride 를
            # 받지 않는다.
            batch = prepare_observation_for_inference(
                {
                    "observation.images.gripper": np.ascontiguousarray(image),
                    "observation.state": np.asarray(state, dtype=np.float32),
                },
                device, task=self._task)

            t0 = time.monotonic()
            with torch.inference_mode():
                chunk = self._policy.predict_action_chunk(self._pre(batch))
                # 후처리기는 한 스텝씩 (B, action_dim) 을 받는다 — 청크를
                # 통째로 넣으면 안 된다. 여기서 정규화가 풀려 원시 카운트가
                # 된다(policy_server._predict_action_chunk 와 같은 방식).
                steps = torch.stack(
                    [self._post(chunk[:, i, :]) for i in range(chunk.shape[1])],
                    dim=1).squeeze(0)
            took = time.monotonic() - t0

            actions = [[int(round(float(v))) for v in row]
                       for row in steps.detach().cpu().numpy()]
            now = time.monotonic()
            if not self._engaged:
                self._engage(state, now)
            self._player.submit(actions, now)
            self.get_logger().info(
                f"추론 {took * 1000:.0f}ms · 청크 {len(actions)}스텝 "
                f"({len(actions) / self._send_hz:.1f}초 분량)")

    def _engage(self, state, now):
        """추종을 켠다 — 팔로워가 현재 자세를 기준점으로 잡게 만든다.

        먼저 현재 실측 자세를 ChunkPlayer 에 심고 epoch 를 올린다. 이
        순서를 바꾸면 첫 패킷이 기준점 없이 나가 팔이 튄다."""
        self._player.prime(state)
        self._epoch += 1
        self._engaged = True
        self.get_logger().info(f"추종 시작 #{self._epoch} — 기준 자세 {state}")

    # ── 송신 ────────────────────────────────────────────────────────────────

    def _send_loop(self):
        period = 1.0 / self._send_hz
        next_at = time.monotonic()
        last_reason = ""
        while not self._stop.is_set():
            now = time.monotonic()
            tick = self._player.tick(now)

            if tick.reason and tick.reason != last_reason:
                self.get_logger().warn(f"추종 해제 — {tick.reason}")
                last_reason = tick.reason
            elif not tick.reason:
                last_reason = ""

            engaged = bool(tick.engaged) and self._engaged and not self._dry_run
            pos = tick.counts if tick.counts else [None] * episode_spec.JOINT_COUNT
            self._seq += 1
            # 베이스는 건드리지 않는다 — 이 시연의 범위는 팔이다. 정지
            # 방향을 계속 보내므로 팔로워가 바퀴를 굴리지 않는다.
            self._sock.sendto(
                encode(self._seq, self._epoch, engaged, pos, (0.0, 0.0, 0.0), 0.0),
                self._addr)

            next_at += period
            sleep = next_at - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_at = time.monotonic()      # 밀렸으면 따라잡지 않는다

    def start(self):
        self._sender.start()
        self._infer.start()

    def shutdown(self):
        self._stop.set()
        # 마지막으로 해제를 알린다 — 팔로워는 토크를 유지한 채 그 자리에 선다.
        for _ in range(3):
            self._seq += 1
            self._sock.sendto(
                encode(self._seq, self._epoch, False,
                       [None] * episode_spec.JOINT_COUNT, (0.0, 0.0, 0.0), 0.0),
                self._addr)
        if self._player.clamped_total:
            self.get_logger().warn(
                f"슬루로 잘린 관절 명령 {self._player.clamped_total}건 — "
                "정책이 물리적으로 불가능한 속도를 요구하고 있습니다")


def main(args=None):
    rclpy.init(args=args)
    node = SmolvlaPolicyNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
