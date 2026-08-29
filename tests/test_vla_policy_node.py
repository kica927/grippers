"""정책 노드의 구조 계약 (2026-08-30, VLA 스트레치).

rclpy 의존이라 맥에서 import 할 수 없다 — 이 저장소가 arm_driver_node 나
grasp_cycle 에 쓰는 방식대로 AST 와 소스로 읽는다.

여기서 지키는 것은 **팔을 움직이는 경로가 하나로 유지되는가**다. 정책이
자기만의 구동 경로를 뚫는 순간, 팔로워 수신기에 쌓아 둔 안전장치(관절
한계·슬루·데드맨·토크 유지)가 전부 우회된다.
"""

import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
NODE = ROOT / "ros2_ws" / "src" / "grippers_vla" / "grippers_vla" / "smolvla_policy_node.py"
CONVERTER = ROOT / "tools" / "vla" / "bag_to_lerobot.py"


def _source():
    return NODE.read_text(encoding="utf-8")


def _tree():
    return ast.parse(_source(), filename=str(NODE))


def _docstrings(tree):
    """docstring 노드 집합. 설명문에 적힌 말을 코드로 착각하면 안 된다 —
    '이것을 쓰지 않는다'고 써 둔 문장이 '쓴다'로 잡힌다."""
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef,
                             ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node, clean=False)
            if doc is not None:
                out.add(doc)
    return out


def _code_strings(tree):
    """docstring 을 뺀, 실제로 값으로 쓰이는 문자열 리터럴."""
    docs = _docstrings(tree)
    return [n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and n.value not in docs]


def _imported(tree):
    """import 로 끌어온 최상위 모듈 이름."""
    names = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            names.update(a.name.split(".")[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom) and n.module:
            names.add(n.module.split(".")[0])
    return names


# ── 구동 경로는 하나뿐이다 ─────────────────────────────────────────────────


def test_서보를_직접_열지_않는다():
    """/dev/soarm 을 직접 열면 팔로워 수신기의 안전장치를 통째로 우회한다.
    게다가 시리얼 포트는 한 프로세스만 열 수 있어, 사람이 손으로 이어받는
    경로까지 막힌다."""
    tree = _tree()

    assert "driver_sdk" not in _imported(tree)
    assert not [s for s in _code_strings(tree) if "/dev/" in s], \
        "장치 경로를 값으로 들고 있으면 안 된다"
    assert "STS3215Driver" not in {
        n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}


def test_텔레옵_규약으로_보낸다():
    """팔로워가 이해하는 유일한 형식이다."""
    src = _source()

    assert "from teleop_protocol import" in src
    assert "encode(" in src


def test_슬루와_붙들기는_공용_로직을_쓴다():
    """여기서 다시 구현하면 테스트된 규칙과 조용히 갈라진다."""
    src = _source()

    assert "action_chunk" in src
    assert "ChunkPlayer" in src


# ── 처음 켤 때 안전 ────────────────────────────────────────────────────────


def test_dry_run_이_기본값이다():
    """첫 실행에서 팔이 움직이면 안 된다. 켜는 것은 사람이 명시적으로
    한 번 더 해야 한다."""
    tree = _tree()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, "attr", "") == "declare_parameter"
                and node.args
                and getattr(node.args[0], "value", "") == "dry_run"):
            assert node.args[1].value is True
            return
    raise AssertionError("dry_run 파라미터 선언을 못 찾았다")


def test_dry_run_이면_engaged_를_켜지_않는다():
    src = _source()

    assert "not self._dry_run" in src, \
        "dry_run 이 engaged 계산에 반영돼야 한다"


def test_종료할_때_해제를_알린다():
    """알리지 않고 죽으면 팔로워는 0.4초 데드맨이 걸릴 때까지 마지막
    목표를 붙들고 있다."""
    tree = _tree()
    shutdown = next(n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == "shutdown")
    body = ast.dump(shutdown)

    assert "sendto" in body
    assert "False" in body


def test_베이스를_움직이지_않는다():
    """이 시연의 범위는 팔이다. 텔레옵 패킷은 팔과 베이스를 같이 싣기
    때문에, 정지 방향을 명시하지 않으면 남은 값이 실려 나갈 수 있다."""
    assert "(0.0, 0.0, 0.0), 0.0" in _source()


# ── 해제된 뒤 다시 잡을 수 있는가 ──────────────────────────────────────────
#
# 팔로워는 해제되면 `if not self.tracking: return` 에 갇힌다. 같은 epoch 로
# engaged=True 를 아무리 보내도 다시 안 잡는다 — **새 epoch 만이 latch 를
# 다시 건다**(follower_teleop_node.on_packet). 추론이 느려 한 번 해제되면
# 그 뒤로 패킷은 계속 나가는데 팔은 영영 안 움직인다.


def test_해제되면_다시_engage_하도록_표시한다():
    src = _source()

    assert "if not tick.engaged and self._engaged:" in src
    assert "self._engaged = False" in src


def test_engage_가_epoch_을_올린다():
    """epoch 를 안 올리면 팔로워가 기준점을 다시 안 잡는다."""
    tree = _tree()
    engage = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "_engage")

    assert "_epoch" in ast.dump(engage)
    assert "prime" in ast.dump(engage)


def test_engage_가_지금_자세를_읽는다():
    """추론이 몇 초 걸렸으므로 추론 직전의 자세는 낡았을 수 있다. 낡은
    자세로 기준을 잡으면 팔로워가 그 차이만큼 팔을 튕긴다."""
    tree = _tree()
    engage = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "_engage")

    assert "_state" in ast.dump(engage), "현재 상태를 다시 읽어야 한다"


# ── 속도를 섞지 않는가 ─────────────────────────────────────────────────────


def test_액션_속도와_송신_속도가_따로_있다():
    """청크의 한 스텝은 데이터셋 fps 짜리 움직임이고, 송신 속도는 데드맨이
    정한다. 하나로 합치면 정책이 배운 속도로 안 움직인다."""
    src = _source()

    assert "action_hz" in src
    assert "send_hz" in src


def test_기본_액션_속도를_공용_모듈에서_가져온다():
    """노드가 15 를 직접 쓰면 데이터셋 fps 가 바뀔 때 한 곳만 고쳐진다."""
    src = _source()

    assert "ACTION_HZ_DEFAULT = action_chunk.ACTION_HZ_DEFAULT" in src


# ── 학습과 추론이 같은 세상을 봐야 한다 ────────────────────────────────────


def test_색_순서가_변환기와_같다():
    """학습은 RGB 로 하고 추론은 BGR 로 넣으면, 정책은 색이 뒤집힌 세상을
    본다. 실패가 조용해서 성능 저하로만 보인다."""
    assert "[:, :, ::-1]" in _source()
    assert "[:, :, ::-1]" in CONVERTER.read_text(encoding="utf-8")


def test_관절_정의를_공용_모듈에서_가져온다():
    """노드가 6을 직접 쓰면 관절 수가 바뀔 때 한 곳만 고쳐진다."""
    src = _source()

    assert "episode_spec" in src
    assert "JOINT_COUNT" in src


# ── 패키지 등록 ────────────────────────────────────────────────────────────


def test_실행_파일로_등록돼_있다():
    """등록이 없으면 ros2 run 으로 못 띄운다 — colcon build 는 통과한다."""
    setup = (ROOT / "ros2_ws" / "src" / "grippers_vla" / "setup.py") \
        .read_text(encoding="utf-8")

    assert "smolvla_policy_node = grippers_vla.smolvla_policy_node:main" in setup


def test_lerobot_을_rosdep_에_적지_않는다():
    """rosdep 에 없는 이름을 package.xml 에 적으면 rosdep install 이
    저장소 전체에서 실패한다."""
    pkg = (ROOT / "ros2_ws" / "src" / "grippers_vla" / "package.xml") \
        .read_text(encoding="utf-8")

    assert "<depend>lerobot</depend>" not in pkg
    assert "<depend>torch</depend>" not in pkg


def test_무거운_의존은_늦게_import_한다():
    """모듈 최상단에서 torch 를 끌면, 정책 없이 노드 상태만 보려 할 때도
    수 초가 걸리고 메모리를 먹는다."""
    tree = ast.parse(_source())
    top_level = set()
    for n in tree.body:
        if isinstance(n, ast.Import):
            top_level.update(a.name.split(".")[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom) and n.module:
            top_level.add(n.module.split(".")[0])

    assert "torch" not in top_level
    assert "lerobot" not in top_level
