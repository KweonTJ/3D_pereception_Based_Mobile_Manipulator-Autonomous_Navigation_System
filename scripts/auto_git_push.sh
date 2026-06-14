#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(git rev-parse --show-toplevel)"
cd "$REPO_DIR"

REMOTE="origin"
BRANCH="$(git branch --show-current)"

SSH_REMOTE="git@github-kweontj:KweonTJ/3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System.git"

if [ -z "$BRANCH" ]; then
  echo "[ERROR] detached HEAD 상태입니다."
  exit 1
fi

echo "======================================"
echo "[AUTO GIT PUSH]"
echo "REPO   : $REPO_DIR"
echo "BRANCH : $BRANCH"
echo "REMOTE : $(git remote get-url "$REMOTE")"
echo "======================================"

echo "[1] remote를 KweonTJ SSH로 고정"
git remote set-url "$REMOTE" "$SSH_REMOTE"

echo "[2] git pull 방식 설정"
git config pull.rebase true
git config rebase.autoStash true

echo "[3] 현재 상태 확인"
git status --short

echo "[4] 변경사항 자동 커밋"
if [ -n "$(git status --porcelain)" ]; then
  git add -A
  git commit -m "auto: turtlebot update $(date '+%Y-%m-%d %H:%M:%S')"
else
  echo "commit할 변경사항 없음"
fi

echo "[5] remote fetch"
git fetch "$REMOTE"

echo "[6] origin/$BRANCH 위로 rebase"
git pull --rebase "$REMOTE" "$BRANCH"

echo "[7] push"
git push -u "$REMOTE" "$BRANCH"

echo "======================================"
echo "[DONE] auto commit + rebase + push 완료"
echo "======================================"
