"""gripper_cam_publisher_node — /dev/gripper_cam(raw V4L2)을 ROS2 Image 토픽으로 퍼블리시.

그리퍼캠은 ROS2 드라이버 없이 raw V4L2 장치로만 존재한다. live_yolo_demo.py
같은 ROS2 구독 기반 도구를 depth_cam뿐 아니라 gripper_cam에도 그대로 붙이기
위한 최소 브리지다.

2026-08-25: 이 노드가 /dev/gripper_cam의 **유일한 소유자**가 됐다.
예전에는 perception_node의 confirm_grasp 서비스가 __init__ 시점에 같은
장치를 열어 fd를 계속 쥐고 있어서, 이 노드와 동시에 실행하면 V4L2 경합
(Device or resource busy)이 났다. 그 confirm_grasp 경로는 면적/밝기차로는
파지 성공을 판정할 수 없다는 것이 실측으로 확인돼(빈 그리퍼 닫힘
165990px²가 룩을 문 상태 70384px²보다 컸고, 이탈한 축구공의 diff 1.88이
빈 그리퍼 4.65보다 낮았다) 통째로 제거했다. 파지 판정은 CARRY_IDLE의
load로 한다.

용도는 시연·디버깅 화면이다 — 팔을 내리면 depth 카메라가 가려져 거리
표시가 사라지는 구간에서 사람이 눈으로 확인할 유일한 시야다. 거리 추정에는
쓰지 않는다(오버헤드 ArUco 좌표가 그 역할을 한다).
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEVICE_DEFAULT = "/dev/gripper_cam"
OUTPUT_TOPIC_DEFAULT = "gripper_cam/image_raw"
WIDTH = 640
HEIGHT = 480
WARMUP_FRAMES = 5
PUBLISH_PERIOD_SEC = 1.0 / 15.0


class GripperCamPublisherNode(Node):
    def __init__(self):
        super().__init__("gripper_cam_publisher_node")
        self.declare_parameter("device", DEVICE_DEFAULT)
        self.declare_parameter("output_topic", OUTPUT_TOPIC_DEFAULT)

        self._bridge = CvBridge()
        device = self.get_parameter("device").value
        output_topic = self.get_parameter("output_topic").value

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"그리퍼캠을 열지 못했습니다: {device}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        for _ in range(WARMUP_FRAMES):
            self._cap.grab()

        self._publisher = self.create_publisher(Image, output_topic, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)
        self.get_logger().info(f"gripper_cam_publisher_node ready: {device} -> {output_topic}")

    def _on_timer(self):
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warn("gripper_cam: 프레임 읽기 실패")
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)

    def destroy_node(self):
        self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperCamPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
