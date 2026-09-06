#!/bin/bash
# grippers Pi bringup/teardown 감시 스냅샷 — 2026-09-06, 사용자 지시.
#
# 왜 만들었나: run_mission.py 를 Enter 로 끝낸 뒤 다시 실행했을 때만 모터가
# 안 움직이는 증상이 반복된다는 보고가 있었다. 조사해 보니 bringup_now.sh
# 의 사전 점검(`ps -eo cmd | grep -E '...arm_driver_node...'`)이 실제
# 프로세스 명령줄과 다른 문자열을 찾고 있었다 — ROS 그래프 이름은
# `/arm_driver_node` 지만 실제 실행 파일은 `arm_driver` 뿐이라
# "arm_driver_node" 는 `ps` 출력에 **절대 나타나지 않는다**(2026-09-06
# 실기로 확인: `ps -eo cmd | grep arm_driver_node` 는 항상 매치 0건).
# 그 결과 이전 세션이 남긴 arm_driver 프로세스가 있어도 bringup_now.sh는
# "이미 떠 있는 노드 없음"으로 착각하고 그 위에 새 arm_driver 를 또
# 띄운다 — 둘이 같은 시리얼 포트(/dev/soarm)를 두고 충돌한다. 이 버그는
# bringup_now.sh 에서 이미 고쳤다(주석 참고). 이 스크립트는 그런 사고가
# 다시 생기더라도 사후에 바로 알아볼 수 있게, bringup/teardown 전후 상태를
# 하나의 로그(/tmp/bringup_audit.log)에 계속 남긴다.
#
# kill 은 절대 하지 않는다 — 읽기 전용이라 사람 확인 없이 아무 때나
# 실행해도 안전하다(Claude Code 가 직접 실행해도 된다).
#
# 사용: node_snapshot.sh <라벨>   예) node_snapshot.sh BEFORE_BRINGUP

set -eo pipefail

LABEL="${1:-snapshot}"
AUDIT_LOG=/tmp/bringup_audit.log
TS=$(date -u '+%Y-%m-%dT%H:%M:%SZ')

# defunct(좀비)도 일부러 포함한다 — 좀비 자체는 무해하지만, 같은 이름의
# 노드가 반복해서 좀비로 남는다는 건 이전 launch 종료가 깨끗하지 않았다는
# 신호라 그대로 보여야 한다.
NODE_PATTERN='ros_robot_controller|odom_publisher|ekf_node|joint_state_publisher|ascamera_node|arm_driver|perception_node|robot_state_publisher|mission_orchestrator|depth_cam_rotate_node'

{
  echo "===== [$TS] $LABEL ====="

  echo "-- ps (실제 프로세스, defunct 포함) --"
  ps -eo pid,ppid,etimes,stat,cmd | grep -E "$NODE_PATTERN" | grep -v grep || echo "(없음)"

  echo "-- ros2 node list (ROS 그래프 기준) --"
  if ros2_out=$(source /opt/ros/humble/setup.bash 2>/dev/null && ros2 node list 2>&1); then
    echo "$ros2_out"
  else
    echo "(ros2 daemon 응답 없음 또는 오류: $ros2_out)"
  fi

  echo "-- /tmp/bringup.pgid --"
  if [ -f /tmp/bringup.pgid ]; then
    pgid=$(cat /tmp/bringup.pgid)
    echo "내용: $pgid"
    if kill -0 -- "-$pgid" 2>/dev/null; then
      echo "-> 이 PGID 는 지금 살아 있다"
    else
      echo "-> ⚠️ 이 PGID 는 이미 죽었는데 파일이 안 지워졌다(고아 파일) — stop_bringup.sh 가 다음에 이걸 보고 헛되이 '정리할 게 없다'고 판단하지 않는지 확인할 것"
    fi
  else
    echo "(파일 없음 — bringup_now.sh 로 띄운 적이 없거나, stop_bringup.sh 가 이미 정리했다)"
  fi

  zombies=$(ps -eo pid,ppid,stat,cmd | awk '$3 ~ /^Z/ {$1=$1; print}' | grep -v grep || true)
  if [ -n "$zombies" ]; then
    echo "-- ⚠️ 좀비(defunct) 프로세스 --"
    echo "$zombies"
  fi

  echo "====================================="
  echo ""
} >> "$AUDIT_LOG"

echo "[node_snapshot] 기록: $AUDIT_LOG (라벨: $LABEL)"
tail -n 40 "$AUDIT_LOG"
