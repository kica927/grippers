"""LeRobot 0.4.4 와 맞물리는 자리 (2026-08-30, VLA 스트레치).

lerobot 은 이 저장소 밖의 라이브러리이고, **하드웨어 없이는 실행으로 확인할
수 없다.** 그래서 2026-08-30 에 lerobot 0.4.4 휠을 받아 소스를 직접 읽고
확인한 사실을 여기에 못 박는다. 나중에 누가 이 호출들을 '정리'하면 그때
바로 걸린다 — 안 그러면 실기 당일, 학습을 다 끝낸 뒤에 안다.

## 그날 실제로 틀려 있던 것 두 가지

**① 정규화가 정책 밖으로 나가 있었다.** 0.4.x 는 전·후처리기가 따로다.
정책만 열고 predict_action_chunk 를 부르면 원시 카운트(약 2048)가 정규화
없이 들어가고, 정규화된 값이 그대로 나온다. 그걸 서보 목표로 보내면 팔이
엉뚱한 곳으로 간다. **둘 다 예외 없이 조용히 틀린다.**

**② add_frame 에 task 인자가 없다.** `task` 는 프레임 딕셔너리 안의 키다.
인자로 주면 TypeError, 아예 없으면 ValueError 다.
"""

import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
NODE = ROOT / "ros2_ws" / "src" / "grippers_vla" / "grippers_vla" / "smolvla_policy_node.py"
CONVERTER = ROOT / "tools" / "vla" / "bag_to_lerobot.py"
SPEC = ROOT / "tools" / "vla" / "episode_spec.py"
README = ROOT / "tools" / "vla" / "README.md"

VERIFIED_VERSION = "0.4.4"


# ── ① 정책은 전·후처리기와 함께 열린다 ────────────────────────────────────


def test_전후처리기를_같이_연다():
    """lerobot/async_inference/policy_server.py:159 와 같은 순서.
    정책만 열면 정규화가 통째로 빠진다."""
    src = NODE.read_text(encoding="utf-8")

    assert "make_pre_post_processors" in src
    assert "pretrained_path=path" in src


def test_관측에_전처리기를_통과시킨다():
    """전처리기가 정규화·토큰화·배치를 한다."""
    src = NODE.read_text(encoding="utf-8")

    assert "self._pre(batch)" in src


def test_액션을_스텝마다_후처리한다():
    """후처리기는 (B, action_dim) 을 받는다 — 청크 (B, T, D) 를 통째로
    넣으면 안 된다. 여기서 정규화가 풀려 원시 카운트가 된다."""
    src = NODE.read_text(encoding="utf-8")

    assert "self._post(chunk[:, i, :])" in src


def test_관측_준비를_직접_만들지_않는다():
    """prepare_observation_for_inference 가 /255 · CHW · 배치차원 ·
    디바이스 이동 · task/robot_type 삽입을 한 번에 한다. 손으로 흉내내면
    그중 하나가 빠져도 모른다."""
    src = NODE.read_text(encoding="utf-8")

    assert "prepare_observation_for_inference" in src


def test_이미지를_연속_배열로_넘긴다():
    """BGR->RGB 를 [:, :, ::-1] 로 하면 stride 가 음수가 되는데,
    torch.from_numpy 는 음수 stride 를 받지 않는다."""
    src = NODE.read_text(encoding="utf-8")

    assert "np.ascontiguousarray(image)" in src


# ── ② 데이터셋 프레임 규약 ────────────────────────────────────────────────


def test_task_는_프레임_안의_키다():
    """lerobot 0.4.4 datasets/lerobot_dataset.py:1171 — add_frame(self, frame).
    인자가 없다."""
    src = CONVERTER.read_text(encoding="utf-8")

    assert '"task": args.task' in src
    assert "add_frame(frame)" in src
    assert "add_frame(frame, task=" not in src


def test_add_frame_을_인자_하나로_부른다():
    tree = ast.parse(CONVERTER.read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "add_frame"]

    assert calls, "add_frame 호출을 못 찾았다"
    for call in calls:
        assert len(call.args) == 1 and not call.keywords


def test_피처_이름이_lerobot_철자와_같다():
    """datasets/utils.py:661 은 'channels'(복수)를 쓰고, :724 에서 names[2]
    를 보고 (h,w,c) -> (c,h,w) 로 바꾼다."""
    src = SPEC.read_text(encoding="utf-8")

    assert '"names": ["height", "width", "channels"]' in src


# ── 체크포인트가 이 팔의 것인지 ────────────────────────────────────────────


def test_적재할_때_액션_차원을_확인한다():
    """틀린 체크포인트를 줘도 추론은 그냥 돈다. 관절 수가 안 맞는 것을
    움직이고 나서 알면 늦다 — 그 시점엔 이미 팔이 잡혀 있다.

    _get_action_chunk 가 내부 패딩(max_action_dim=32)을 데이터셋 차원으로
    다시 자르므로(modeling_smolvla.py:295), config.action_feature 가 곧
    우리가 받는 차원이다."""
    src = NODE.read_text(encoding="utf-8")

    assert "action_feature" in src
    assert "JOINT_COUNT" in src


def test_적재할_때_카메라_이름을_확인한다():
    """이름이 다르면 prepare_images 가 '이미지 피처가 전부 없다'로 예외를
    던지는데, 그건 첫 추론 시점이다."""
    src = NODE.read_text(encoding="utf-8")

    assert "image_features" in src


def test_카메라_이름이_변환기와_같다():
    """학습 데이터셋에 'gripper' 로 들어갔으면 추론도 그 이름이어야 한다."""
    node = NODE.read_text(encoding="utf-8")
    conv = CONVERTER.read_text(encoding="utf-8")

    assert 'IMAGE_KEY = "gripper"' in node
    assert 'image_keys = ["gripper"]' in conv


def test_적재_검사가_예외를_던진다():
    """경고만 하면 사람이 지나친다."""
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    check = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == "_check_policy_shape")

    assert [n for n in ast.walk(check) if isinstance(n, ast.Raise)]


def test_적재_검사를_실제로_부른다():
    """검사 함수가 있어도 안 부르면 아무 일도 안 일어난다. 그리고 안 부르는
    쪽으로 바꿔도 다른 테스트는 전부 통과한다 — 그래서 호출을 따로 못
    박는다."""
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    load = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == "_load_policy")
    calls = [n for n in ast.walk(load)
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "_check_policy_shape"]

    assert calls, "_load_policy 가 _check_policy_shape 를 불러야 한다"


# ── 확인한 버전을 적어 둔다 ────────────────────────────────────────────────


def test_확인한_버전이_문서에_적혀_있다():
    """0.3.x 에서 정규화가 프로세서로 옮겨 갔다. 버전을 내리면 위의 호출이
    전부 깨지므로, 무엇을 보고 짰는지 남겨야 한다."""
    text = README.read_text(encoding="utf-8")

    assert VERIFIED_VERSION in text
    assert "lerobot" in text


def test_무거운_의존을_최상단에서_끌지_않는다():
    """정책 노드는 lerobot 없이도 import 되어야 한다 — 구조 테스트와
    ros2 노드 등록이 그 위에서 돈다."""
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    top = set()
    for n in tree.body:
        if isinstance(n, ast.Import):
            top.update(a.name.split(".")[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom) and n.module:
            top.add(n.module.split(".")[0])

    assert "lerobot" not in top
    assert "torch" not in top
