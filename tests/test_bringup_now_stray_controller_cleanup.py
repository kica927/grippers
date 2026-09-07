"""bringup_now.sh가 부팅 자동 실행 ros_robot_controller를 STALE 체크보다
먼저 치우는지 (2026-09-08, sysy009 vla-dp 브랜치 커밋 38fdabd 이식).

## 배경

Pi는 부팅하면 ros_robot_controller를 자동으로 띄운다. bringup도
controller/odom_publisher.launch.py로 자체 컨트롤러를 띄우므로, 그대로
두면 두 프로세스가 같은 시리얼 포트를 문다.

증상이 고약하다 — 소프트웨어는 끝까지 정상으로 보인다. 노드 다 뜨고
cmd_vel도 나가고 set_motor도 정상값이 찍히는데 바퀴만 안 돈다. sysy009가
2026-09-07 vla-dp 브랜치에서 이걸로 몇 시간을 태웠고, 재부팅 뒤 중복
컨트롤러를 없애자마자 바퀴가 돌았다(커밋 38fdabd).

이 저장소(kica927/baseline_mission)의 bringup_now.sh는 원래 "kill은 안
한다 — 뭔가 남아 있으면 stop_bringup.sh를 먼저 돌리라고 알려주고 멈춘다"는
원칙이었다. 그런데 stop_bringup.sh는 /tmp/bringup.pgid(bringup_now.sh가
직접 띄운 것만 추적)가 없으면 그대로 실패한다 — 부팅 자동 실행분은 이
파일에 없으므로, 기존 안내를 따라가면 stop_bringup.sh도 실패하는 막다른
길이었다. 그래서 이 좁은 경우(bringup.launch가 전혀 안 떠 있을 때의
ros_robot_controller만)에 한해 STALE 체크보다 먼저 자동 정리하도록
예외를 뒀다.

`rclpy`/`launch` 의존성이 없는 순수 bash라 소스 텍스트를 그대로 읽어
검사한다(다른 bash 도구 테스트가 없어 sysy009 vla-dp 브랜치의
tests/test_run_vla_mission_args.py와 같은 방식을 그대로 따른다)."""

import pathlib

SCRIPT = (pathlib.Path(__file__).resolve().parent.parent / "tools" / "ops"
          / "bringup_now.sh")


def _body() -> str:
    text = SCRIPT.read_text(encoding="utf-8")
    return "\n".join(line for line in text.splitlines()
                      if not line.lstrip().startswith("#"))


def test_부팅_자동_실행_컨트롤러를_먼저_치운다():
    body = _body()
    assert "ros_robot_controller" in body, "자동 실행분을 정리하는 코드가 없다"
    # bringup.launch 유무를 먼저 확인해야 한다 — 이미 떠 있는 bringup의
    # 컨트롤러까지 죽이면 원래 원칙(kill은 사람이 시킨 것만)이 깨진다.
    guard = body.index('pgrep -f "bringup.launch"')
    kill = body.index('pkill -9 -f "ros_robot_controller"')
    assert guard < kill, "bringup 유무를 먼저 확인해야 한다"


def test_정리가_STALE_점검보다_먼저다():
    """STALE 점검이 이 프로세스를 먼저 걸러 stop_bringup.sh로 보내면,
    PGID 파일이 없어 stop_bringup.sh가 실패하는 막다른 길이 된다."""
    body = _body()
    kill = body.index('pkill -9 -f "ros_robot_controller"')
    stale_check = body.index("STALE=$(ps -eo cmd")
    assert kill < stale_check, "STALE 점검보다 먼저 치워야 그 점검이 막다른 길로 안 샌다"


def test_launch가_이미_떠있으면_건드리지_않는다():
    body = _body()
    # if ! pgrep ... 조건 블록 안에 pkill이 있어야 한다 — bringup.launch가
    # 떠 있을 때는 이 블록 자체를 안 타야 한다는 뜻이다.
    guard_if = body.index('if ! pgrep -f "bringup.launch"')
    kill = body.index('pkill -9 -f "ros_robot_controller"')
    fi = body.index("\nfi", kill)
    assert guard_if < kill < fi, "pkill이 bringup.launch 부재 조건 블록 밖에 있다"
