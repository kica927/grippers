"""bringup.launch.py가 라이다 하드웨어 노드를 다시 띄우지 않는지 (2026-09-06,
사용자 지시: "괜히 문제 생길 수도 있으니 라이다는 빼자").

## 배경

LIDAR_INSERT_CHECK_ENABLED(domain/task/baseline_constants.py)는 이미
False라 INSERT 최종 판정은 라이다를 안 본다. 하지만 BaselineCarryState의
접근 중 실시간 "너무 가깝다" 체크(domain/task/corrections.
retreat_if_too_close)는 그 스위치와 무관하게 항상 돌고 있었고, 그게
나이트 실기(2026-09-06)에서 INSERT_BLOCKED를 20번 연속 낸 원인이었다.

물리 라이다 드라이버 노드 자체를 안 띄우면 `/scan_raw`에 아무도 publish
하지 않으므로 Ros2Lidar.basket_face()가 항상 "스캔 없음"을 돌려주고, 그
아래 모든 라이다 의존 분기가 자연히 건드려지지 않는다 — Python 코드
(Ros2Lidar·basket_lidar_align·preconditions.check_insert·corrections.
retreat_if_too_close 등)는 하나도 지우지 않고 기록으로 남겼다(사용자
지시). bringup.launch.py의 `lidar_launch`도 그 위 battery_buzzer_monitor
와 같은 방식 — 정의는 주석으로 남기고 실제 반환 목록에서만 뺐다.

`rclpy`/`launch` 의존성 때문에 이 파일을 직접 import할 수 없어(다른
launch 파일 테스트가 없는 이유와 같다), 소스를 AST로 읽는다."""

import ast
import pathlib

REAL_PATH = (pathlib.Path(__file__).resolve().parent.parent / "ros2_ws" / "src"
             / "grippers_bringup" / "launch" / "bringup.launch.py")


def _parse():
    return ast.parse(REAL_PATH.read_text(encoding="utf-8"), filename=str(REAL_PATH))


def test_launch_setup의_반환_목록에_lidar_launch가_없다():
    tree = _parse()
    launch_setup = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "launch_setup")

    returns = [n for n in ast.walk(launch_setup) if isinstance(n, ast.Return)]
    assert returns, "launch_setup에 return문이 없다 — 함수 구조가 바뀌었다"

    offenders = []
    for ret in returns:
        if not isinstance(ret.value, ast.List):
            continue
        for elt in ret.value.elts:
            if isinstance(elt, ast.Name) and elt.id == "lidar_launch":
                offenders.append(elt.lineno)

    assert not offenders, (
        f"lidar_launch가 launch_setup의 반환 목록에 다시 들어갔다 (줄 {offenders}) "
        "— 라이다 하드웨어 노드가 다시 뜬다. 되살리려면 lidar_launch 정의부의 "
        "주석 처리된 원래 대입도 함께 복원해야 한다(지금은 None이라, 목록에만 "
        "다시 넣으면 launch가 None 액션 때문에 깨진다)")


def test_lidar_launch는_None으로_비활성화돼_있다():
    tree = _parse()
    launch_setup = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "launch_setup")

    assigns = [
        n for n in ast.walk(launch_setup)
        if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "lidar_launch" for t in n.targets)
    ]
    assert len(assigns) == 1, (
        f"lidar_launch 대입이 {len(assigns)}번 있다 — 정확히 하나(None)여야 한다")
    assert isinstance(assigns[0].value, ast.Constant) and assigns[0].value.value is None, (
        "lidar_launch가 None이 아닌 값으로 되살아났다 — 의도한 변경이면 이 "
        "테스트를 지우고, 실수면 되돌릴 것")
