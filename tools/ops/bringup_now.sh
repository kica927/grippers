#!/bin/bash
# grippers 실기 bringup — 컨테이너 안 비-인터랙티브(docker exec bash)에서도
# 항상 되게 만든 버전. (2026-09-03, 배경 설명)
#
# ~/ros2_ws/.zshrc 가 exec_shell.sh(zsh, 진짜 TTY)로 들어갈 때는
# need_compile/DEPTH_CAMERA_TYPE 을 자동으로 export 해 주지만(2026-09-01
# 조치), 그건 zsh 로그인 셸에서만 sourcing된다. `docker exec ... bash -c`
# 로 비-인터랙티브하게 들어가면 .zshrc 체인 자체를 안 타서 이 env가
# 하나도 안 잡히고, 특히 third_party_ws(ascamera) 오버레이를 안 sourcing
# 하면 "package 'ascamera' not found"로 launch 전체가 죽는다 — 그것도
# base/odom/ekf 는 이미 뜬 뒤라 다음 재시도가 중복 세대를 만든다.
#
# 이 스크립트는 .robotrc 가 하는 sourcing 전부를 bash에서 그대로 재현하고,
# 시작 전에 이미 떠 있는 노드가 있는지부터 확인해서 중복 기동을 막는다.
# kill 은 안 한다 — 뭔가 남아 있으면 stop_bringup.sh 를 먼저 돌리라고
# 알려주고 여기서 멈춘다.

set -eo pipefail
# -u(미설정 변수 금지)는 안 쓴다 — ROS의 setup.bash 들이 AMENT_TRACE_
# SETUP_FILES 같은 변수를 먼저 체크 없이 참조해서(벤더 코드, 우리가 못
# 고침) -u 아래서는 소싱 자체가 죽는다.

# <defunct>(좀비)는 제외 — 이미 죽은 프로세스라 자원을 안 쥐고 있고,
# 부모가 reap 하면 곧 사라진다. 여기서 걸러야 할 건 "진짜 살아서 포트를
# 쥐고 있는" 프로세스뿐이다.
#
# ⚠️ 2026-09-06 실기로 확인한 버그를 여기서 고쳤다: 이 목록은 원래
# "arm_driver_node"와 "ekf_node"를 포함했는데, 실제 `ps` 명령줄에는 그
# 문자열이 **절대 나오지 않는다**. arm_driver_node는 ROS 그래프 노드
# 이름일 뿐이고 실행 파일은 `arm_driver`다(`ps -eo cmd | grep
# arm_driver_node`는 항상 매치 0건이었다 — Pi 실기로 재현·확인). ekf_node는
# grippers_bringup 자체가 애초에 안 띄운다(bringup.launch.py 주석 참고 —
# imu_calib 부재로 controller.launch.py 전체를 못 써서 odom_publisher만
# 직접 포함하고 EKF는 뺐다). 그래서 이 STALE 점검은 이전 세션이 남긴
# arm_driver 프로세스를 절대 못 잡고, 그 위에 새 arm_driver 를 또 띄워
# 둘이 같은 시리얼 포트(/dev/soarm)를 두고 충돌하는 사고로 이어질 수
# 있었다 — run_mission.py 를 한 번 끝내고 다시 실행했을 때만 모터(팔)가
# 안 움직이는 증상과 일치한다.
STALE=$(ps -eo cmd | grep -E \
  'ros_robot_controller|odom_publisher|joint_state_publisher|ascamera_node|arm_driver|perception_node|robot_state_publisher' \
  | grep -v grep | grep -v defunct || true)

/grippers/tools/ops/node_snapshot.sh BEFORE_BRINGUP > /dev/null || true

if [ -n "$STALE" ]; then
  echo "이미 떠 있는 노드가 있습니다 — 먼저 stop_bringup.sh 를 돌리세요:"
  echo "$STALE"
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export need_compile=False
export DEPTH_CAMERA_TYPE=ascamera
export LD_LIBRARY_PATH="/home/ubuntu/ros2_ws/src/third_party/ascamera/libs/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"

# 세 워크스페이스를 전부 sourcing 한다 — 하나라도 빠지면 그 안 패키지가
# "not found"로 조용히 빠진다(2026-09-03 실기로 확인, 아래 순서 중요):
#   1) /opt/ros/humble            — ROS2 자체
#   2) /home/ubuntu/ros2_ws       — MentorPi 벤더 스택(controller, bringup,
#      ascamera 런치 포함 — 순정 `bringup` 패키지가 여기 있다. 이건 LD19/
#      제스처/라인추적 같은 범용 데모 스택이지 그리퍼스 전용이 아니다)
#   3) /home/ubuntu/third_party_ros2/third_party_ws  — ascamera 드라이버
#   4) /ros2_ws                   — **그리퍼스 프로젝트 전용** 워크스페이스
#      (grippers 저장소의 ros2_ws/ 를 컨테이너에 바인드 마운트한 것).
#      grippers_arm/grippers_perception/grippers_bringup 이 전부 여기 있다.
#      `grippers_bringup`(이 워크스페이스) 와 `bringup`(2번 워크스페이스)은
#      **이름이 겹치지만 완전히 다른 패키지**다 — `bringup`으로 실행하면
#      벤더 데모 스택만 뜨고 arm_driver_node/perception_node는 아예 안
#      뜬다(2026-09-03에 이걸로 한 번 헤맸다). 그리퍼스 미션을 돌리려면
#      반드시 grippers_bringup 이어야 한다.
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /ros2_ws/install/setup.bash

LOG=/tmp/bringup.log
PGID_FILE=/tmp/bringup.pgid

# 2026-09-07, 2차 회전정지 사고 후속 — $LOG는 컨테이너 자체의 오버레이
# 파일시스템(/tmp) 안에 있어서, 컨테이너가 재시작되면(재부팅 등) 통째로
# 사라진다. 정지 안 되는 사고의 최후 수단이 하필 재부팅/전원차단인데,
# 그 순간 가장 필요한 증거가 같이 지워지는 구조였다(2차 사고 때 실제로
# 이걸로 로그를 통째로 잃었다 — bringup.log도 dmesg도 둘 다 컨테이너/
# 커널 재시작에 안 살아남는 곳에만 있었다). 호스트에 바인드 마운트된
# /shared(재부팅에도 살아남는다, docker inspect로 확인)에도 실시간으로
# 같은 내용을 미러링해 둔다 — 메인 launch의 프로세스 그룹(stop_bringup.sh가
# 이걸로 죽인다)과는 별개의 tail 프로세스라, 이게 실패해도 bringup 자체엔
# 영향이 없다.
PERSIST_DIR=/shared/bringup_logs
mkdir -p "$PERSIST_DIR" 2>/dev/null || true
PERSIST_LOG="$PERSIST_DIR/bringup_$(date -u +%Y%m%dT%H%M%SZ).log"
touch "$LOG"
pkill -f "tail -n \+1 -F $LOG" 2>/dev/null || true
( tail -n +1 -F "$LOG" > "$PERSIST_LOG" 2>/dev/null & ) 2>/dev/null || true

echo "기동 중 — 로그: $LOG (호스트 사본: $PERSIST_LOG), 정지: stop_bringup.sh"
setsid ros2 launch grippers_bringup bringup.launch.py \
  use_fake_base:=false use_fake_arm:=false use_fake_perception:=false \
  host_ip:="${1:-192.168.0.9}" \
  > "$LOG" 2>&1 &

LAUNCH_PID=$!
# setsid 로 새 프로세스 그룹의 리더가 됐으므로 PGID == PID.
echo "$LAUNCH_PID" > "$PGID_FILE"
echo "launch PID/PGID = $LAUNCH_PID (기록: $PGID_FILE)"

# 이 시점엔 노드들이 아직 초기화 중이라 "준비 완료" 스냅샷은 아니다 —
# PGID가 실제로 기록됐는지, launch 직후 시점의 프로세스 상태가 어떤지만
# 남긴다. 노드가 다 뜬 뒤의 스냅샷은 test_ready.sh(3/3 뒤)가 따로 찍는다.
/grippers/tools/ops/node_snapshot.sh AFTER_LAUNCH_TRIGGERED > /dev/null || true
