"""gripper_cam_publisher_node — /dev/gripper_cam(raw V4L2)을 ROS2 토픽으로 퍼블리시.

그리퍼캠은 ROS2 드라이버 없이 raw V4L2 장치로만 존재한다. 이 노드가 그것을
토픽으로 올려서, ROS2 구독 기반 도구(live_yolo_demo.py)와 rosbag2 녹화,
그리고 VLA 정책의 관측 입력이 같은 자리에서 영상을 받게 한다.

## 두 토픽을 내는 이유

    gripper_cam/image_raw              sensor_msgs/Image            bgr8 원본
    gripper_cam/image_raw/compressed   sensor_msgs/CompressedImage  JPEG

**원본만으로는 녹화가 안 된다.** bgr8 640x480 을 15Hz 로 흘리면 분당 약
0.8GB 다. tools/teleop/record_demo.sh 가 여유 2GB 미만이면 중단하도록 되어
있으니, 원본을 그대로 넣으면 2~3분 만에 걸린다. VLA 시연은 한 에피소드가
수십 초씩 수십 번이라 그 형태로는 수집 자체가 성립하지 않는다.

**압축만으로도 안 된다.** live_yolo_demo.py 를 비롯한 기존 도구가 Image 를
구독한다. 그래서 둘 다 낸다. 원본이 필요 없는 실행에서는 `publish_raw:=false`
로 끄면 CPU 와 DDS 대역을 아낀다.

## cv_bridge 를 쓰지 않는다

⚠️ 2026-08-23 실기 확인: cv_bridge 의 컴파일된 확장(cv_bridge_boost.so)이
numpy 1.x ABI 로 빌드돼 있어 이 환경의 numpy 2.x 와 맞지 않는다
(`AttributeError: _ARRAY_API not found` -> 세그폴트, ultralytics/torch 가
numpy 2.x 를 끌어온 뒤 발생). 그때 perception_node 와 depth_cam_rotate_node
에서는 걷어냈는데(3e1a207) 이 파일만 남아 있었다 — 저장소에서 cv_bridge 를
실제로 import 하던 마지막 노드였다.

bgr8 uint8 배열을 Image 메시지로 직접 채우는 것은 표준 절차라 cv_bridge 없이
안전하다. JPEG 는 cv2.imencode 가 바로 바이트를 준다.

## 같은 장치를 두 프로그램이 못 연다

perception_node 의 `perception/confirm_grasp` 서비스도 같은 /dev/gripper_cam
을 독점으로 연다. 동시에 뜨면 V4L2 경합(Device or resource busy)이다.

기동 실패를 "카메라가 없다"로 오해하기 쉬워서, 열기에 실패하면 그 가능성을
메시지에 같이 적는다. 미션은 그 서비스를 부르지 않으므로(호출부 없음)
실기에서는 이 노드 쪽을 켜는 것이 맞다.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

DEVICE_DEFAULT = "/dev/gripper_cam"
OUTPUT_TOPIC_DEFAULT = "gripper_cam/image_raw"
WIDTH = 640
HEIGHT = 480
WARMUP_FRAMES = 5
PUBLISH_PERIOD_SEC = 1.0 / 15.0

# 시연 수집용이라 화질보다 용량이다. 85 는 JPEG 에서 육안 열화가 거의 없는
# 하한선으로 통용되는 값이고, 640x480 한 장이 대략 40~60KB 로 떨어진다
# (원본 921,600 바이트의 5% 안팎).
JPEG_QUALITY_DEFAULT = 85


def _image_msg_from_bgr(img, stamp):
    """BGR cv2 배열을 Image 메시지로 바꾼다 — cv_bridge를 쓰지 않는다.

    depth_cam_rotate_node._image_msg_from_bgr 과 같은 방식이다. 그쪽이
    2026-08-23 실기에서 이미 검증된 경로다."""
    msg = Image()
    msg.header.stamp = stamp
    msg.height, msg.width = img.shape[0], img.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


def _compressed_msg_from_bgr(img, stamp, quality):
    """BGR 배열을 JPEG CompressedImage 로. 인코딩 실패는 None."""
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.format = "jpeg"
    msg.data = buf.tobytes()
    return msg


class GripperCamPublisherNode(Node):
    def __init__(self):
        super().__init__("gripper_cam_publisher_node")
        self.declare_parameter("device", DEVICE_DEFAULT)
        self.declare_parameter("output_topic", OUTPUT_TOPIC_DEFAULT)
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("jpeg_quality", JPEG_QUALITY_DEFAULT)

        device = self.get_parameter("device").value
        output_topic = self.get_parameter("output_topic").value
        self._publish_raw = bool(self.get_parameter("publish_raw").value)
        self._quality = int(self.get_parameter("jpeg_quality").value)

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"그리퍼캠을 열지 못했습니다: {device}\n"
                "  · 장치가 없다면 udev 규칙(99-gripper-cam.rules)과 USB 연결 확인\n"
                "  · 장치가 있는데 실패한다면 perception_node 의 confirm_grasp 가\n"
                "    같은 장치를 이미 열고 있을 수 있습니다 — 둘은 동시에 못 뜹니다"
            )
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        for _ in range(WARMUP_FRAMES):
            self._cap.grab()

        self._pub_raw = (self.create_publisher(Image, output_topic, 10)
                         if self._publish_raw else None)
        self._pub_jpeg = self.create_publisher(
            CompressedImage, f"{output_topic}/compressed", 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)

        which = "원본+압축" if self._publish_raw else "압축만"
        self.get_logger().info(
            f"gripper_cam_publisher_node ready: {device} -> {output_topic} ({which})")

    def _on_timer(self):
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warn("gripper_cam: 프레임 읽기 실패")
            return
        stamp = self.get_clock().now().to_msg()
        if self._pub_raw is not None:
            self._pub_raw.publish(_image_msg_from_bgr(frame, stamp))
        jpeg = _compressed_msg_from_bgr(frame, stamp, self._quality)
        if jpeg is None:
            self.get_logger().warn("gripper_cam: JPEG 인코딩 실패")
            return
        self._pub_jpeg.publish(jpeg)

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
