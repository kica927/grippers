#!/bin/bash
# grippers 실기 bringup 정지 — 개별 PID를 하나씩 찾아 kill하지 않는다.
# bringup_now.sh 가 setsid 로 띄운 launch 프로세스 그룹 전체에 SIGINT를
# 한 번 보내면, ros2 launch 자신의 정상 종료 경로(각 노드에 SIGINT ->
# 응답 없으면 SIGTERM)를 그대로 타서 base/odom/ekf/카메라/팔/인식까지
# 전부 한 번에 정리된다 — 이게 ros2 launch 를 끄는 정석 방법이다.
#
# 이 스크립트는 kill 을 실제로 하므로 **사용자가 직접 실행**해야 한다
# (모터 제어 프로세스에 영향을 줄 수 있는 명령은 Claude Code 가 대필하지
# 않는다는 이 프로젝트의 표준 원칙 — CHANGES_2026-09-02.md 참고).

set -euo pipefail

PGID_FILE=/tmp/bringup.pgid

if [ ! -f "$PGID_FILE" ]; then
  echo "$PGID_FILE 가 없습니다 — bringup_now.sh 로 띄운 게 아니면 이 스크립트로는 못 끕니다."
  echo "그 경우 ps -eo pid,cmd | grep -E 'ros_robot_controller|odom_publisher|joint_state_publisher|ascamera_node|arm_driver|perception_node|robot_state_publisher' 로 직접 찾아서 정리하세요."
  # PGID 파일이 없는데 노드는 살아 있는 상태(2026-09-06 실기로 실제
  # 목격) — 이 상태 자체가 "다음 bringup 이 자기가 뭘 죽여야 하는지
  # 모른다"는 신호라 반드시 기록해 둔다.
  /grippers/tools/ops/node_snapshot.sh STOP_REQUESTED_NO_PGID > /dev/null || true
  exit 1
fi

/grippers/tools/ops/node_snapshot.sh BEFORE_TEARDOWN > /dev/null || true

PGID=$(cat "$PGID_FILE")
echo "프로세스 그룹 $PGID 에 SIGINT 전송..."
kill -INT -- "-$PGID" 2>/dev/null || true

for i in $(seq 1 10); do
  if ! kill -0 -- "-$PGID" 2>/dev/null; then
    echo "정상 종료됨"
    rm -f "$PGID_FILE"
    /grippers/tools/ops/node_snapshot.sh AFTER_TEARDOWN_CLEAN > /dev/null || true
    exit 0
  fi
  sleep 1
done

echo "10초 안에 안 죽어서 SIGKILL 보냅니다"
kill -9 -- "-$PGID" 2>/dev/null || true
rm -f "$PGID_FILE"
# SIGKILL 로 넘어갔다는 건 정상 종료 경로(ros2 launch 의 각 노드 SIGINT
# 전파)를 못 탔다는 뜻이다 — 자식이 reap 안 된 채 좀비로 남을 가능성이
# 이 경로에서 특히 높다(2026-09-06, 좀비 다발 확인의 배경). 그래서 이
# 경로에서도 반드시 스냅샷을 남긴다.
/grippers/tools/ops/node_snapshot.sh AFTER_TEARDOWN_SIGKILL > /dev/null || true
