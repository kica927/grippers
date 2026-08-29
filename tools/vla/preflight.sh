#!/usr/bin/env bash
# preflight.sh — VLA 시연 수집 전 점검. 아무것도 바꾸지 않는다 (읽기 전용).
#
# 컨테이너 안에서:
#   export ROS_DOMAIN_ID=21
#   bash /grippers/tools/vla/preflight.sh
#
# 하지 않는 것: 노드를 죽이지 않는다 · /cmd_vel 을 발행하지 않는다 ·
# 파라미터를 바꾸지 않는다 · 저장소에 쓰지 않는다 · 서보에 쓰지 않는다.

set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

fail=0; warn_n=0
ok()   { printf '  \033[32m OK \033[0m %s\n' "$1"; }
warn() { printf '  \033[33m ?? \033[0m %s\n' "$1"; warn_n=$((warn_n+1)); }
bad()  { printf '  \033[31m XX \033[0m %s\n' "$1"; fail=$((fail+1)); }

has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
rate_of()   { timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -m1 -o 'average rate: [0-9.]*' || true; }

echo "=== 1. ROS 환경 ==="
[ "${ROS_DOMAIN_ID:-}" = "21" ] && ok "ROS_DOMAIN_ID=21" \
  || bad "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-없음} — 21 이어야 합니다"
command -v ros2 >/dev/null && ok "ros2 있음" || bad "ros2 없음 — setup.zsh 를 source 하세요"

echo
echo "=== 2. 저장소를 건드리지 않았는가 ==="
if command -v git >/dev/null && [ -d "$ROOT/.git" ]; then
  branch=$(git -C "$ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null)
  dirty=$(git -C "$ROOT" status --porcelain 2>/dev/null | wc -l | tr -d ' ')
  [ "$branch" = "kica927/smolVLA-version" ] && ok "브랜치 $branch" \
    || warn "브랜치 $branch — VLA 수집은 kica927/smolVLA-version 입니다"
  [ "$dirty" = "0" ] && ok "작업트리 깨끗" || warn "변경 ${dirty}건 — 의도한 것인지 보세요"
else
  warn "git 정보를 못 읽었습니다"
fi

echo
echo "=== 3. 배포된 코드가 검증한 그 코드인가 ==="
if python3 "$ROOT/tools/vla/selftest.py"; then :; else bad "자체 점검 실패 — 위 항목을 보세요"; fi

echo
echo "=== 4. 팔 캘리브레이션 (읽기만 합니다) ==="
if pgrep -f 'arm_driver|follower_teleop' >/dev/null 2>&1; then
  warn "팔을 잡고 있는 프로세스가 있어 건너뜁니다 (시리얼 포트는 하나만 열립니다)"
elif [ ! -e /dev/soarm ]; then
  warn "/dev/soarm 이 없습니다 — 팔 전원과 USB 를 확인하세요"
else
  out=$(python3 "$ROOT/tools/arm/restore_taught_offsets.py" 2>&1)
  if echo "$out" | grep -q "일치합니다"; then
    warn "지금 팔은 **교시(베이스라인) 캘리브레이션**입니다."
    printf '       VLA 시연을 이대로 찍으면 베이스라인 자세로 학습됩니다.\n'
    printf '       의도한 것이면 그대로 두고, 아니면 LeRobot 캘리브레이션을 먼저 하세요.\n'
  else
    ok "지금 팔은 VLA(LeRobot) 캘리브레이션입니다 — 수집에 맞습니다"
  fi
fi

echo
echo "=== 5. 그리퍼캠 ==="
if [ -e /dev/gripper_cam ]; then ok "/dev/gripper_cam 있음"
else bad "/dev/gripper_cam 없음 — USB 와 udev 규칙(99-gripper-cam.rules) 확인"; fi

if ros2 node list 2>/dev/null | grep -q gripper_cam_publisher; then
  ok "gripper_cam_publisher_node 떠 있음"
else
  bad "gripper_cam_publisher_node 가 없습니다 — 영상 없이 찍으면 학습 데이터가 안 됩니다"
  printf '       ros2 run grippers_perception gripper_cam_publisher_node\n'
fi
if ros2 node list 2>/dev/null | grep -q perception_node; then
  warn "perception_node 도 떠 있습니다 — confirm_grasp 가 같은 장치를 잡으면 둘 중 하나만 열립니다"
fi

for t in /gripper_cam/image_raw/compressed /gripper_cam/image_raw; do
  if has_topic "$t"; then ok "$t $(rate_of "$t")"; else
    [ "$t" = "/gripper_cam/image_raw/compressed" ] \
      && bad "$t 없음 — 녹화가 조용히 비게 됩니다" || warn "$t 없음 (화면 확인용, 없어도 수집은 됩니다)"
  fi
done

echo
echo "=== 6. 텔레옵 상태 토픽 ==="
for t in /teleop/follower_present /teleop/follower_counts /teleop/engaged /cmd_vel; do
  if has_topic "$t"; then ok "$t"; else
    [ "$t" = "/teleop/follower_present" ] \
      && bad "$t 없음 — observation.state 가 비어 학습 데이터가 안 됩니다" \
      || warn "$t 없음 (텔레옵을 아직 안 켰다면 정상입니다)"
  fi
done
printf '       텔레옵을 안 켰으면 위 세 개가 없는 것이 정상입니다.\n'
printf '       켠 뒤 다시 돌려 follower_present 가 나오는지 꼭 보세요.\n'

echo
echo "=== 7. 디스크와 녹화 시간 ==="
out_dir="${OUT_DIR:-/grippers/recordings}"
avail_mb=$(df -Pm "$(dirname "$out_dir")" 2>/dev/null | awk 'NR==2{print $4}')
avail_mb=${avail_mb:-0}
if [ "$avail_mb" -ge 4096 ]; then ok "여유 ${avail_mb} MB"
elif [ "$avail_mb" -ge 2048 ]; then warn "여유 ${avail_mb} MB — 자주 옮기세요"
else bad "여유 ${avail_mb} MB — record_demo.sh 가 2048MB 미만이면 멈춥니다"; fi

if has_topic /gripper_cam/image_raw/compressed; then
  bw=$(timeout 8 ros2 topic bw /gripper_cam/image_raw/compressed 2>/dev/null \
       | grep -m1 -o '^[0-9.]*[KM]*B/s' || true)
  if [ -n "$bw" ]; then
    mb_min=$(python3 - "$bw" <<'PY'
import re, sys
m = re.match(r"([0-9.]+)([KM]?)B/s", sys.argv[1])
v = float(m.group(1)) * {"": 1e-6, "K": 1e-3, "M": 1.0}[m.group(2)]
print(f"{v * 60:.0f}")
PY
)
    ok "영상 ${bw} → 약 ${mb_min} MB/분"
    [ "$mb_min" -gt 0 ] && ok "지금 여유로 약 $((avail_mb / mb_min))분 녹화 가능 (영상만 기준)"
  else
    warn "대역폭을 못 쟀습니다 — 발행이 없거나 ros2 topic bw 가 조용합니다"
  fi
fi

echo
echo "=== 8. 추론 단계 준비물 (수집에는 없어도 됩니다) ==="
python3 - <<'PY' 2>/dev/null || echo "  ?? lerobot 없음 — 수집·변환에는 필요 없습니다. 추론 전에 설치하세요"
import lerobot
v = getattr(lerobot, "__version__", "알 수 없음")
mark = "OK" if v.startswith("0.4.") else "??"
print(f"  {mark} lerobot {v}" + ("" if v.startswith("0.4.") else "  <- 0.4.4 로 검증했습니다"))
PY
python3 -c "import cv2; print('  OK cv2', cv2.__version__)" 2>/dev/null \
  || echo "  XX cv2 없음 — 변환기가 JPEG 를 못 풉니다"
python3 -c "import rosbag2_py; print('  OK rosbag2_py')" 2>/dev/null \
  || echo "  ?? rosbag2_py 없음 — 변환은 ROS 환경에서 하세요"

echo
if [ "$fail" -gt 0 ]; then
  printf '\033[31m판정: 막힘 — 위의 XX %d건을 먼저 해결하세요.\033[0m\n' "$fail"
  exit 1
fi
[ "$warn_n" -gt 0 ] && printf '\033[33m판정: 수집 가능 (확인할 것 %d건)\033[0m\n' "$warn_n" \
                    || printf '\033[32m판정: 수집 가능\033[0m\n'
exit 0
