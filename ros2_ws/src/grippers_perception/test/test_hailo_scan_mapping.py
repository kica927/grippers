"""hailo_scan_mapping.object_class_for_hailo_id() 순수 로직 테스트.

rclpy/cv2/hailo_platform 없이도 돈다 — perception_node.py 자체는 rclpy를
무조건 import해서 ROS2 없이는 임포트가 안 되지만, 이 매핑 로직은 그 파일에서
뽑아냈으므로(2026-08-22) 여기서만 순수 pytest로 검증한다."""

import pytest
from grippers_perception.hailo_scan_mapping import (
    HAILO_CLASS_NAMES,
    HAILO_CLASS_TO_OBJECT_CLASS,
    hailo_bbox_to_frame_xyxy,
    object_class_for_hailo_id,
)


@pytest.mark.parametrize(
    "class_name,expected",
    [
        ("knight", "CHESS_PIECE"),
        ("queen", "CHESS_PIECE"),
        ("rook", "CHESS_PIECE"),
        ("box", "GABE"),
        ("soccer", "GABE"),
        ("star", "GABE"),
    ],
)
def test_known_classes_map_to_expected_object_class(class_name, expected):
    class_id = HAILO_CLASS_NAMES.index(class_name)
    assert object_class_for_hailo_id(class_id) == expected


def test_destination_box_class_is_excluded():
    """"container"는 목적지 상자 클래스로 추정된다 — 목적지는 YOLO로 찾지
    않으므로(2026-08-23 확정 미션 명세서, 좌표 하드코딩) 바닥 스캔 후보에서
    제외돼야 한다. "box"는 장난감(GABE)이라 더 이상 여기 포함되지 않는다."""
    class_id = HAILO_CLASS_NAMES.index("container")
    assert object_class_for_hailo_id(class_id) is None


def test_out_of_range_class_id_returns_none():
    """HEF가 바뀌어 클래스 수가 달라져도 죽지 않고 제외로 접는다."""
    assert object_class_for_hailo_id(len(HAILO_CLASS_NAMES)) is None
    assert object_class_for_hailo_id(-1) is None


def test_every_mapped_class_name_is_a_real_hailo_class():
    """HAILO_CLASS_TO_OBJECT_CLASS의 키가 HAILO_CLASS_NAMES에 없는 이름으로
    오타 나는 걸 잡는다 — 오타가 나면 조용히 매핑이 안 먹는다."""
    assert set(HAILO_CLASS_TO_OBJECT_CLASS).issubset(set(HAILO_CLASS_NAMES))


def test_mapped_values_are_known_object_classes():
    """domain.values.ObjectClass는 GABE/CHESS_PIECE 둘뿐이다 — 이 문자열이
    그 두 값과 어긋나면 Ros2Perception이 만든 Detection이 도메인에서 조용히
    거부되거나 잘못 해석된다."""
    assert set(HAILO_CLASS_TO_OBJECT_CLASS.values()) <= {"GABE", "CHESS_PIECE"}


# --- hailo_bbox_to_frame_xyxy() 왕복 검증 (2026-09-06) ---
#
# perception_node._letterbox(frame, size)와 정확히 반대 연산이어야 한다.
# cv2 없이 그 함수의 수학(비율 유지 리사이즈 + 중앙 패딩)만 손으로 재현해
# "원본 bbox -> 레터박스 캔버스 좌표(정규화) -> 이 함수로 역변환하면
# 원래 bbox로 돌아오는가"를 검증한다 — observe_target()이 Hailo로 옮겨가며
# 이 변환이 하나라도 틀리면 GRASP 거리/좌우 판정이 조용히 다 어긋난다.


def _letterbox_forward(bbox_xyxy, canvas_size, orig_h, orig_w):
    """`perception_node._letterbox`가 원본 좌표를 캔버스의 어디에 놓는지
    순수 산술로 재현한다(테스트 전용 — cv2를 쓰지 않는다)."""
    x1, y1, x2, y2 = bbox_xyxy
    scale = min(canvas_size / orig_h, canvas_size / orig_w)
    resized_w = round(orig_w * scale)
    resized_h = round(orig_h * scale)
    x0 = (canvas_size - resized_w) // 2
    y0 = (canvas_size - resized_h) // 2
    return (
        (x0 + x1 * scale) / canvas_size,
        (y0 + y1 * scale) / canvas_size,
        (x0 + x2 * scale) / canvas_size,
        (y0 + y2 * scale) / canvas_size,
    )


@pytest.mark.parametrize(
    "orig_h,orig_w,canvas_size,bbox_xyxy",
    [
        # 정사각형 원본 — 패딩 없음(scale=1, x0=y0=0)인 단순 케이스.
        (640, 640, 640, (100.0, 50.0, 200.0, 150.0)),
        # 가로가 긴 원본(800x400) — 위아래 패딩이 생기는 케이스
        # (본문 docstring 예시와 동일한 수치).
        (400, 800, 640, (100.0, 50.0, 200.0, 150.0)),
        # 세로가 긴 원본(480x270 depth_cam 스트림 흔한 종횡비) — 좌우 패딩.
        (480, 270, 640, (10.0, 200.0, 260.0, 400.0)),
    ],
)
def test_hailo_bbox_to_frame_xyxy_round_trips_through_letterbox(
    orig_h, orig_w, canvas_size, bbox_xyxy
):
    x1, y1, x2, y2 = bbox_xyxy
    norm_x1, norm_y1, norm_x2, norm_y2 = _letterbox_forward(
        bbox_xyxy, canvas_size, orig_h, orig_w)
    # Hailo NMS-by-class 출력 순서: [y_min, x_min, y_max, x_max, score]
    # (tools/hailo/live_yolo_demo.py의 draw_detections와 동일 관례).
    det = [norm_y1, norm_x1, norm_y2, norm_x2, 0.87]

    (out_x1, out_y1, out_x2, out_y2), score = hailo_bbox_to_frame_xyxy(
        det, canvas_size, orig_h, orig_w)

    assert out_x1 == pytest.approx(x1, abs=0.6)
    assert out_y1 == pytest.approx(y1, abs=0.6)
    assert out_x2 == pytest.approx(x2, abs=0.6)
    assert out_y2 == pytest.approx(y2, abs=0.6)
    assert score == pytest.approx(0.87)


def test_hailo_bbox_to_frame_xyxy_preserves_score_type():
    """score가 numpy float32 등이어도 순수 float로 나와야 한다 —
    아니면 이후 산술(중앙값 계산 등)에서 타입이 섞여 놀랄 수 있다."""
    _, score = hailo_bbox_to_frame_xyxy(
        [0.1, 0.1, 0.2, 0.2, 0.5], 640, 480, 640)
    assert isinstance(score, float)
