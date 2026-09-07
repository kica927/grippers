#!/bin/bash
# grippers "테스트 준비" 한 방에 — 2026-09-03, 사용자 지시로 정리.
#
# 이 전까지는 코드 배포 확인 -> EEPROM 비교 -> bringup 을 매번 손으로
# 하나씩 ssh 왕복하며 진행했고, bringup 쪽에서만 env/워크스페이스/패키지명
# 문제로 세 번 헤맸다(그 root cause는 bringup_now.sh 에 이미 반영돼 있다).
# 이 스크립트는 그 세 단계를 이어 붙인 것뿐이고, kill 은 전혀 하지 않는다
# — 그래서 Claude Code 가 사람 확인 없이 바로 실행해도 안전하다.
#
# 실행 위치: Pi 홈 (컨테이너 밖). git 동기화는 호스트에서, EEPROM 확인과
# bringup 은 docker exec 로 컨테이너 안에서 한다.
#
# run_mission.py 실행(9단계)과 stop_bringup.sh(kill 포함)는 이 스크립트에
# 없다 — 그건 항상 사용자가 직접 한다.

set -eo pipefail

REPO=~/docker/shared/grippers

echo "=== 1/3 코드 상태 확인 ==="
cd "$REPO"
git fetch --all --quiet
git status -sb
# 2026-09-07: origin(grippers-intel 팀 저장소) 대비로 비교하던 것을
# personal-mirror(kica927/grippers)로 고쳤다. 이 프로젝트 규칙(메모리
# grippers-git-sync-personal-mirror-source)이 "항상 personal-mirror에서
# 동기화, origin은 절대 안 씀"인데 이 줄만 origin 기준이라 어긋나 있었다.
# 그 결과 personal-mirror에만 push된 커밋(배터리 부저 문턱 7200mV 변경,
# 10390e8)이 origin엔 없어 "이미 최신입니다"로 잘못 보고됐고, Pi가 실제로는
# 1커밋 뒤처진 채(구버전 7800mV 문턱) 계속 돌았다 — 실기에서 부저가 새
# 문턱보다 훨씬 위(7798mV)에서 울려서 발견됐다.
BEHIND=$(git rev-list --count HEAD..personal-mirror/kica927/baseline_mission 2>/dev/null || echo 0)
if [ "$BEHIND" != "0" ]; then
  echo "원격이 $BEHIND 커밋 앞서 있습니다 — ff-only 로 받습니다."
  git merge --ff-only personal-mirror/kica927/baseline_mission
  git log --oneline -3
else
  echo "이미 최신입니다."
fi

echo ""
echo "=== 2/3 EEPROM 캘리브레이션 비교 (읽기 전용) ==="
docker exec -u ubuntu IntelPi bash -lc \
  'cd /grippers && python3 tools/arm/restore_taught_offsets.py' \
  || echo "⚠️ 비교 도구가 비정상 종료했습니다 — 위 출력을 확인하세요."
echo "불일치가 있으면 --apply --yes 는 사람이 직접 실행해야 합니다(토크 꺼짐, 팔이 내려옵니다)."

echo ""
echo "=== 3/3 bringup ==="
docker exec -u ubuntu IntelPi bash -c '/grippers/tools/ops/bringup_now.sh 192.168.0.9'

echo ""
echo "=== 노드 확인 ==="
docker exec -u ubuntu IntelPi bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'

echo ""
echo "=== 스냅샷 기록 (/tmp/bringup_audit.log, 컨테이너 안) ==="
docker exec -u ubuntu IntelPi bash -c '/grippers/tools/ops/node_snapshot.sh AFTER_BRINGUP_READY' \
  || echo "⚠️ 스냅샷 기록 실패 — node_snapshot.sh 가 배포됐는지 확인하세요."
