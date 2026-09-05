#!/usr/bin/env python3
"""Hailo vs CPU YOLO scan_floor 추론 속도 비교 (2026-09-06, 사용자 지시).

## 왜 이 스크립트가 필요한가

perception_node.py의 scan_floor는 Hailo/CPU YOLO 두 백엔드 로딩
(`_load_hailo_model`/`_load_cpu_yolo_model`)과 실제 추론 메서드
(`_scan_floor_detections_hailo`/`_scan_floor_detections_cpu_yolo`)를 이미
다 갖고 있다. 그런데 이 둘을 실제로 부르는 ROS 서비스가 아직 없다 —
`scan_floor_enabled` 게이트까지 다 있는데 정작 그 서비스 콜백 자체가
`create_service`로 등록돼 있지 않다(코드를 직접 확인함, 2026-09-06 — "구조
검증"만 된 상태라는 뜻이고, perception_node.py가 스스로 밝히는
"find_box/measure_opening/monitor_clearance: NOT IMPLEMENTED" 목록에는
scan_floor가 없어서 자칫 이미 살아 있는 걸로 오해하기 쉽다).

그래서 "Hailo 켜서 CPU YOLO만 쓸 때보다 얼마나 빨라졌는지" 비교는
perception_node를 실제로 띄우는 것만으로는 아직 할 수 없다. 이 스크립트는
tools/hailo/live_yolo_demo.py와 같은 패턴 — 프로덕션 경로에 편입하지 않는
독립 실행 노드 — 로, 같은 카메라 스트림에서 프레임 하나당 순수 추론
소요시간(ms)만 백엔드별로 재서 콘솔과 CSV에 남긴다.

## 쓰는 법

    python3 scan_floor_bench.py --backend hailo --frames 100
    python3 scan_floor_bench.py --backend cpu   --frames 100   # "안 썼을 때" 재현

같은 물체를 같은 자리에 두고 각각 돌린 뒤, 콘솔에 찍히는 평균/중앙값/
최소/최대 ms와 --frames 만큼 쌓인 CSV(아래 CSV_DIR)를 비교한다.

⚠️ 한 프로세스에서 두 백엔드를 동시에 켜지 않는다 — Hailo VDevice는
프로세스당 하나뿐이라(live_yolo_demo.py 상단 경고와 같은 이유) 같이
띄우면 서로 간섭해 시간이 왜곡된다. --backend hailo로 한 번 끝내고
완전히 종료(Ctrl+C)한 뒤에 --backend cpu로 다시 실행할 것.

⚠️ 이 스크립트가 재는 것은 "모델 한 번 돌리는 데 걸린 시간"만이다.
`_scan_floor_detections_cpu_yolo`가 실제로 쓰는 다중 프레임 합의
(`floor_consensus.confirmed_tracks`, CONSENSUS_N_FRAMES장 수집)나 거리
계산(`_approach_pose_m`)은 포함하지 않는다 — 그건 두 백엔드가 공유하지
않는 후처리라 "백엔드 자체가 얼마나 빠른가"를 가리는 잡음이기 때문이다.
CPU 부하/발열까지 같이 보고 싶으면 이 스크립트를 돌리는 동안 별도
터미널에서 `vcgencmd measure_temp`나 `top`을 같이 띄워 둘 것 — 이
스크립트 안에서는 안 잰다(같은 프로세스 안에서 재면 측정 자체가 CPU를
더 쓴다).
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from grippers_perception.perception_node import (
    CPU_YOLO_MODEL_PATH_DEFAULT,
    HAILO_HEF_PATH_DEFAULT,
    HAILO_SCORE_THRESHOLD,
    _bgr_from_image_msg,
)

IMAGE_TOPIC = "depth_cam/rgb/image_rotated"

# /tmp는 컨테이너 자체 파일시스템이라 재시작하면 사라진다 — /shared는
# ros_start.sh가 호스트 docker/shared를 그대로 bind mount하는 자리라
# 컨테이너를 새로 띄워도 남는다(tools/hailo/live_yolo_demo.py 상단 주석과
# 같은 이유로 여기 둔다).
CSV_DIR = "/shared/scan_floor_bench"


def _letterbox(frame, size):
    """tools/hailo/live_yolo_demo.py의 letterbox()/perception_node.py의
    `_letterbox()`와 동일 로직 — 비율 유지 정사각형 레터박스."""
    h, w = frame.shape[:2]
    scale = min(size / h, size / w)
    resized = cv2.resize(frame, (round(w * scale), round(h * scale)))
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    y0 = (size - resized.shape[0]) // 2
    x0 = (size - resized.shape[1]) // 2
    canvas[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
    return canvas


class ScanFloorBenchNode(Node):
    def __init__(self, backend: str, n_frames: int, hef_path: str, model_path: str):
        super().__init__("scan_floor_bench_node")
        self._backend = backend
        self._n_frames = n_frames
        self._samples_ms: list[float] = []
        self._done = False

        if backend == "hailo":
            from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice

            params = VDevice.create_params()
            params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
            self._vdevice = VDevice(params)
            self._infer_model = self._vdevice.create_infer_model(hef_path)
            self._infer_model.input().set_format_type(FormatType.UINT8)
            self._configured_model = self._infer_model.configure()
            self._output_shape = self._infer_model.output().shape
            self._input_size = self._infer_model.input().shape[0]
            self.get_logger().info(
                f"[bench] Hailo 모델 로드됨 {hef_path} (입력={self._input_size})")
        else:
            from ultralytics import YOLO

            self._model = YOLO(model_path)
            self.get_logger().info(f"[bench] CPU YOLO 모델 로드됨 {model_path}")

        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        self.get_logger().info(
            f"[bench] backend={backend} frames={n_frames} — {IMAGE_TOPIC} 대기 중")

    def _on_image(self, msg) -> None:
        if self._done:
            return
        frame = _bgr_from_image_msg(msg)

        t0 = time.perf_counter()
        if self._backend == "hailo":
            canvas = _letterbox(frame, self._input_size)
            bindings = self._configured_model.create_bindings()
            bindings.input().set_buffer(np.ascontiguousarray(canvas))
            bindings.output().set_buffer(np.empty(self._output_shape, dtype=np.float32))
            self._configured_model.run([bindings], timeout=1000)
            detections_by_class = bindings.output().get_buffer()
            n_det = sum(
                1 for dets in detections_by_class for det in dets
                if float(det[4]) >= HAILO_SCORE_THRESHOLD)
        else:
            results = self._model.predict(frame, verbose=False)[0]
            n_det = len(results.boxes)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        self._samples_ms.append(elapsed_ms)
        n = len(self._samples_ms)
        self.get_logger().info(f"[bench] {n}/{self._n_frames} {elapsed_ms:.1f}ms 검출 {n_det}개")

        if n >= self._n_frames:
            self._done = True

    def summary(self) -> dict:
        s = self._samples_ms
        return {
            "backend": self._backend,
            "n": len(s),
            "mean_ms": statistics.mean(s) if s else float("nan"),
            "median_ms": statistics.median(s) if s else float("nan"),
            "min_ms": min(s) if s else float("nan"),
            "max_ms": max(s) if s else float("nan"),
            "fps": (1000.0 / statistics.mean(s)) if s else 0.0,
        }

    def write_csv(self) -> str:
        os.makedirs(CSV_DIR, exist_ok=True)
        path = os.path.join(
            CSV_DIR, f"{self._backend}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["frame_idx", "elapsed_ms"])
            for i, v in enumerate(self._samples_ms, start=1):
                writer.writerow([i, f"{v:.3f}"])
        return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["hailo", "cpu"], required=True,
                        help="hailo=오늘 되살린 경로, cpu='안 썼을 때' 재현")
    parser.add_argument("--frames", type=int, default=100,
                        help="이 프레임 수를 채우면 자동 종료(기본 100)")
    parser.add_argument("--hef-path", default=HAILO_HEF_PATH_DEFAULT)
    parser.add_argument("--model-path", default=CPU_YOLO_MODEL_PATH_DEFAULT)
    args = parser.parse_args()

    rclpy.init()
    node = ScanFloorBenchNode(args.backend, args.frames, args.hef_path, args.model_path)
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.get_logger().info("[bench] 중단됨 — 지금까지 모인 표본으로 요약한다")
    finally:
        summary = node.summary()
        if summary["n"] > 0:
            node.get_logger().info(
                f"[bench] 완료 — backend={summary['backend']} n={summary['n']} "
                f"평균 {summary['mean_ms']:.1f}ms (중앙값 {summary['median_ms']:.1f}, "
                f"최소 {summary['min_ms']:.1f}, 최대 {summary['max_ms']:.1f}) "
                f"~{summary['fps']:.1f}fps"
            )
            csv_path = node.write_csv()
            node.get_logger().info(f"[bench] CSV 저장: {csv_path}")
        else:
            node.get_logger().warn("[bench] 표본이 하나도 안 모였다 — 카메라 토픽 확인할 것")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
