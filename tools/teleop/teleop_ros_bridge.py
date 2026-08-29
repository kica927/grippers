# -*- coding: utf-8 -*-
"""텔레옵의 ROS 쪽 — 베이스 속도 발행 + rosbag2 에 남길 상태 토픽 발행.

발행만 하므로 spin 이 필요 없다 — follower_teleop_node 의 메인 루프는
소켓에서 블로킹하고 있고, publish() 는 그 안에서 바로 호출된다.

토픽:
  /cmd_vel                 Twist            베이스 속도 (컨트롤러가 구독)
  /teleop/leader_counts    Int32MultiArray  리더 원시 카운트(관절 6개)
  /teleop/follower_counts  Int32MultiArray  팔로워에 실제로 내린 목표 카운트
  /teleop/follower_present Int32MultiArray  팔로워가 실제로 가 있는 카운트
  /teleop/engaged          Bool             팔 추종 중 여부
  /teleop/arm_joint_states JointState       사람이 보기 위한 근사 라디안

`follower_counts` 와 `follower_present` 는 다르다. 앞은 **명령**이고 뒤는
**결과**다. 중력 처짐, 물체에 걸림, 슬루 제한에 잘린 이동이 전부 이 둘의
차이로 나타난다. VLA 학습 데이터에서 이 구분은 선택이 아니다 — LeRobot 은
`observation.state`(지금 어디 있나)를 보고 `action`(어디로 갈까)을 내도록
학습하므로, state 자리에 명령을 넣으면 정책이 자기 출력을 관측으로 되먹는
것을 배운다.

주의 — arm_joint_states 의 각도는 **근사값**이다. calibration.json 이 없어
"서보 중앙(2048) = 관절 0도"라고 가정하고 환산한다. 실제 영점은 팔마다
다르므로 이 토픽으로 정밀한 각도를 논하면 안 된다. 정확한 값이 필요하면
같이 녹화되는 *_counts 원시 카운트를 쓸 것.
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, "/third_party/soarm_provided_d/soarm_lab")

import rclpy
from driver_sdk import JOINT_IDS, JOINT_NAMES, STS3215Driver
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray

# odom_publisher_node.app_cmd_vel_callback 의 안전 클램프와 같은 값.
# 더 크게 보내도 어차피 거기서 잘리므로 여기서 맞춰 둔다.
MAX_LIN = 0.2   # m/s
MAX_ANG = 0.5   # rad/s


class RosBridge:
    def __init__(self, node_name: str = "arm_teleop_bridge", cmd_vel_topic: str = "cmd_vel"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.p_cmd = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.p_leader = self.node.create_publisher(
            Int32MultiArray, "teleop/leader_counts", 10)
        self.p_follower = self.node.create_publisher(
            Int32MultiArray, "teleop/follower_counts", 10)
        self.p_present = self.node.create_publisher(
            Int32MultiArray, "teleop/follower_present", 10)
        self.p_engaged = self.node.create_publisher(Bool, "teleop/engaged", 10)
        self.p_js = self.node.create_publisher(
            JointState, "teleop/arm_joint_states", 10)
        self.node.get_logger().info(f"텔레옵 브리지 시작 — 베이스 → /{cmd_vel_topic}")

    # ── 베이스 ───────────────────────────────────────────────────────────────
    def publish_base(self, vec, scale: float):
        x, y, t = vec
        msg = Twist()
        msg.linear.x = float(x) * MAX_LIN * scale
        msg.linear.y = float(y) * MAX_LIN * scale
        msg.angular.z = float(t) * MAX_ANG * scale
        self.p_cmd.publish(msg)

    def stop_base(self):
        """정지는 반드시 성공해야 하는 명령이라 여러 번 낸다 — 한 번 놓치면
        로봇이 마지막 속도로 계속 굴러간다."""
        for _ in range(3):
            self.p_cmd.publish(Twist())

    # ── 팔 상태 ──────────────────────────────────────────────────────────────
    def publish_arm(self, leader_pos: list, follower_targets: dict, engaged: bool,
                    present: dict | None = None):
        # 결측(읽기 실패)은 -1 로 표시한다 — 카운트는 0..4095 라 충돌하지 않는다.
        self.p_leader.publish(Int32MultiArray(
            data=[int(p) if p is not None else -1 for p in leader_pos]))

        fol = [int(follower_targets.get(sid, -1)) for sid in JOINT_IDS]
        self.p_follower.publish(Int32MultiArray(data=fol))
        self.p_engaged.publish(Bool(data=bool(engaged)))

        # 실측 자세는 매 패킷 읽지 않는다(직렬 버스 예산) — 있을 때만 낸다.
        if present is not None:
            self.p_present.publish(Int32MultiArray(
                data=[int(present.get(sid, -1)) for sid in JOINT_IDS]))

        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [
            math.radians(STS3215Driver.position_to_degrees(c)) if c >= 0 else 0.0
            for c in fol
        ]
        self.p_js.publish(js)

    def destroy(self):
        self.node.destroy_node()
