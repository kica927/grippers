#!/usr/bin/env bash
# 시연 rosbag2 녹화. 컨테이너 안에서 실행한다.
#
# 루트 파티션 여유가 크지 않으므로(확인 시점 3.9G) 기본 토픽 집합에서
# 원본 이미지를 뺐다. 카메라를 남기려면 --with-camera 를 주되, 남은 용량을
# 먼저 확인할 것 — 640x480 RGB 30fps 원본은 분당 1.5GB 를 넘는다.
set -euo pipefail

OUT_DIR="${OUT_DIR:-/grippers/recordings}"
WITH_CAMERA=0
MIN_FREE_MB=2048

for a in "$@"; do
  case "$a" in
    --with-camera) WITH_CAMERA=1 ;;
    --help|-h) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "알 수 없는 인자: $a" >&2; exit 2 ;;
  esac
done

source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"

free_mb=$(df -Pm "$(dirname "$OUT_DIR")" | awk 'NR==2{print $4}')
if [ "$free_mb" -lt "$MIN_FREE_MB" ]; then
  echo "여유 공간 ${free_mb}MB — ${MIN_FREE_MB}MB 미만이라 중단합니다." >&2
  echo "정리 후 다시 시도하세요: docker exec IntelPi apt-get clean" >&2
  exit 1
fi

TOPICS=(
  /cmd_vel /odom /tf /tf_static /scan
  /teleop/leader_counts /teleop/follower_counts /teleop/follower_present
  /teleop/engaged /teleop/arm_joint_states
)
if [ "$WITH_CAMERA" -eq 1 ]; then
  # 압축본만 넣는다. bgr8 640x480 15Hz 원본은 분당 0.8GB 라 몇 분이면 디스크가
  # 찬다 — JPEG 는 그 5% 안팎이다.
  found=0
  for t in $(ros2 topic list 2>/dev/null | grep -E 'image_raw/compressed|camera_info'); do
    TOPICS+=("$t")
    found=$((found + 1))
  done

  # 없는 토픽을 주면 ros2 bag 은 오류 없이 그냥 비운다. 그래서 여기서
  # 먼저 본다 — 시연을 다 찍고 나서 영상이 없다는 걸 알면 되돌릴 수 없다.
  if [ "$found" -eq 0 ]; then
    echo "⚠️  --with-camera 를 줬는데 압축 영상 토픽이 하나도 없습니다." >&2
    echo "    gripper_cam_publisher_node 가 떠 있는지 확인하세요:" >&2
    echo "      ros2 run grippers_perception gripper_cam_publisher_node" >&2
    echo "    (perception_node 의 confirm_grasp 가 같은 장치를 잡고 있으면" >&2
    echo "     열리지 않습니다 — 둘은 동시에 못 뜹니다)" >&2
    exit 1
  fi
  if ! printf '%s\n' "${TOPICS[@]}" | grep -q 'gripper_cam'; then
    echo "⚠️  그리퍼캠 압축 토픽이 없습니다 — VLA 시연이면 이대로 찍으면 안 됩니다." >&2
    echo "    계속하려면 Enter, 중단하려면 Ctrl-C" >&2
    read -r _
  fi
fi

mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/demo_$(date +%Y%m%d_%H%M%S)"
echo "녹화 → $BAG"
echo "토픽: ${TOPICS[*]}"
echo "여유: ${free_mb}MB   (Ctrl-C 로 종료)"
exec ros2 bag record -o "$BAG" "${TOPICS[@]}"
