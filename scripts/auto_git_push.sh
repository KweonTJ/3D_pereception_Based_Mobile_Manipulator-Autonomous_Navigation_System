#!/usr/bin/env bash

REPO_DIR="$HOME/turtlebot3_ws/src"
OBSIDIAN_DIR="$HOME/Documents/Obsidian/2026/1학기/UROP/Project"
DOCS_DIR="$REPO_DIR/docs"
BRANCH="main"

sync_obsidian_docs() {
    mkdir -p "$DOCS_DIR"

    rsync -av --delete \
        --include='*/' \
        --include='*.md' \
        --exclude='*' \
        "$OBSIDIAN_DIR/" "$DOCS_DIR/"
}

cd "$REPO_DIR" || exit 1

echo "[auto-git] Watching:"
echo "  repo     : $REPO_DIR"
echo "  obsidian : $OBSIDIAN_DIR"

while true; do
    inotifywait -r -e modify,create,delete,move \
        --exclude '(\.git|build|install|log|__pycache__|\.vscode|.*~|.*\.swp)' \
        "$REPO_DIR" "$OBSIDIAN_DIR"

    echo "[auto-git] Change detected. Waiting 3 seconds..."
    sleep 3

    sync_obsidian_docs

    cd "$REPO_DIR" || exit 1

    if git diff --quiet && git diff --cached --quiet; then
        echo "[auto-git] No meaningful changes."
        continue
    fi

    git add .

    COMMIT_MSG="auto: update $(date '+%Y-%m-%d %H:%M:%S')"

    if git commit -m "$COMMIT_MSG"; then
        git push origin "$BRANCH"
        echo "[auto-git] Pushed to origin/$BRANCH"
    else
        echo "[auto-git] Commit failed. Maybe no changes."
    fi
done
