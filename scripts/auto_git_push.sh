#!/usr/bin/env bash

REPO_DIR="$HOME/turtlebot3_ws/src"
BRANCH="main"

cd "$REPO_DIR" || exit 1

echo "[auto-git] Watching $REPO_DIR ..."

while true; do
    inotifywait -r -e modify,create,delete,move \
        --exclude '(\.git|build|install|log|__pycache__|\.vscode|.*~|.*\.swp)' \
        "$REPO_DIR"

    echo "[auto-git] Change detected. Waiting 3 seconds..."
    sleep 3

    cd "$REPO_DIR" || exit 1

    if git diff --quiet && git diff --cached --quiet; then
        echo "[auto-git] No meaningful changes."
        continue
    fi

    git add .

    COMMIT_MSG="auto: update $(date '+%Y-%m-%d %H:%M:%S')"

    git commit -m "$COMMIT_MSG"

    if [ $? -eq 0 ]; then
        git push origin "$BRANCH"
        echo "[auto-git] Pushed to origin/$BRANCH"
    else
        echo "[auto-git] Commit failed. Maybe no changes."
    fi
done
