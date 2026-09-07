"""Hailo-10H scan_floor 검출 → domain ObjectClass 매핑 (순수 함수).

`perception_node.py`에서 이 매핑 로직만 뽑아냈다 — `perception_node.py`는
`rclpy`를 최상단에서 무조건 import해서 ROS2 없이는 아예 임포트가 안 되고,
그래서 이 매핑 로직도 지금까지 테스트가 하나도 없었다(PR #185 리뷰에서
`scan_floor` 안전 게이트 누락이 리뷰 전에 안 잡힌 이유 중 하나다). 이 파일은
`rclpy`/`cv2`/`hailo_platform`을 전혀 쓰지 않으므로 도메인 테스트와 같은
방식(순수 `pytest`)으로 검증할 수 있다 — `ros2_ws/src/grippers_perception/test/`
참고.

클래스 이름·순서는 HEF 컴파일 당시 metadata.yaml과 반드시 일치해야 한다.
HEF가 바뀌면 `HAILO_CLASS_NAMES`도 같이 바꿀 것.
"""

# metadata.yaml의 names 순서 — HEF가 바뀌면 같이 바꿀 것.
HAILO_CLASS_NAMES = ["container", "knight", "queen", "rook", "box", "soccer", "star"]

# knight/queen/rook은 CHESS_PIECE로, box/soccer/star는 GABE로 매핑한다.
#
# ⚠️ 2026-08-23: 확정 미션 명세서로 "box"가 목적지 상자가 아니라 바닥 위
# 장난감(GABE) 서브클래스임이 확인됐다 — 목적지는 좌표로 하드코딩된
# Destination(LEFT/RIGHT)이지 YOLO가 검출하는 대상이 아니다. "container"는
# 이 모델에만 있는 별도 클래스로, 실제 목적지 상자로 추정되어 여전히
# 바닥 스캔 후보에서 제외한다. "cube"는 애초에 학습 클래스에 없다(미해결).
HAILO_CLASS_TO_OBJECT_CLASS = {
    "knight": "CHESS_PIECE",
    "queen": "CHESS_PIECE",
    "rook": "CHESS_PIECE",
    "box": "GABE",
    "soccer": "GABE",
    "star": "GABE",
    # "container": 목적지 상자로 추정 — 바닥 스캔 후보에서 제외.
}


def object_class_for_hailo_id(class_id: int) -> str | None:
    """Hailo 추론 출력의 class_id(=`HAILO_CLASS_NAMES` 인덱스)를 domain
    `ObjectClass` 이름 문자열("GABE"/"CHESS_PIECE")로 바꾼다.

    매핑이 없으면(범위 밖 class_id, 또는 `HAILO_CLASS_TO_OBJECT_CLASS`에
    의도적으로 안 넣은 클래스) **`None`** — 호출자가 바닥 스캔 후보에서
    제외해야 한다는 신호다. 다른 Perception 계약과 같은 "모르면 제외" 관례."""
    if not 0 <= class_id < len(HAILO_CLASS_NAMES):
        return None
    class_name = HAILO_CLASS_NAMES[class_id]
    return HAILO_CLASS_TO_OBJECT_CLASS.get(class_name)


def hailo_bbox_to_frame_xyxy(det, canvas_size, orig_h, orig_w):
    """Hailo 검출 한 건을 `perception_node._letterbox` 캔버스 좌표계
    (0~1 정규화, 순서 `[y_min, x_min, y_max, x_max, score]` —
    tools/hailo/live_yolo_demo.py의 draw_detections와 동일 관례)에서
    **원본 프레임의 절대 픽셀 xyxy**로 되돌린다. `(scale, x0, y0)` 계산은
    `_letterbox`(비율 유지 리사이즈 + 중앙 패딩)와 정확히 반대 연산이다.

    2026-09-06 — observe_target()을 Hailo로 옮기면서 새로 필요해진
    변환이다. `OBSERVE_MIN_BOTTOM_Y_PX`나 거리 보정 상수
    (`CLASS_DISTANCE_CALIBRATION_SQRT_PX_M`)는 전부 원본 프레임의 절대
    픽셀 좌표를 전제로 튜닝됐다 — 레터박스 캔버스 좌표를 그대로 넘기면
    (a) 패딩 오프셋만큼 어긋나고 (b) 정규화(0~1) 스케일이라 전혀 다른
    값이 되어 두 게이트 모두 조용히 오작동한다. `perception_node.py`가
    아니라 여기 있는 이유는 `object_class_for_hailo_id`와 같다 — rclpy
    없이 순수 pytest로 좌표 왕복(letterbox → 이 함수)이 원래 좌표로
    돌아오는지 검증하기 위함이다(test_hailo_scan_mapping.py 참고)."""
    y_min, x_min, y_max, x_max, score = det
    scale = min(canvas_size / orig_h, canvas_size / orig_w)
    resized_w = round(orig_w * scale)
    resized_h = round(orig_h * scale)
    x0 = (canvas_size - resized_w) // 2
    y0 = (canvas_size - resized_h) // 2
    x1 = (x_min * canvas_size - x0) / scale
    y1 = (y_min * canvas_size - y0) / scale
    x2 = (x_max * canvas_size - x0) / scale
    y2 = (y_max * canvas_size - y0) / scale
    return (x1, y1, x2, y2), float(score)
