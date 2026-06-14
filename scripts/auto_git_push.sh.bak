#!/usr/bin/env bash

REPO_DIR="$HOME/turtlebot3_ws/src"
OBSIDIAN_DIR="$HOME/Documents/Obsidian/2026/1학기/UROP/Project"
DOCS_DIR="$REPO_DIR/docs"
BRANCH="main"
README_PATH="README.md"

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
        --exclude '(\.git|build|install|log|__pycache__|\.vscode|README\.md$|.*~|.*\.swp)' \
        "$REPO_DIR" "$OBSIDIAN_DIR"

    echo "[auto-git] Change detected. Waiting 3 seconds..."
    sleep 3

    sync_obsidian_docs

    cd "$REPO_DIR" || exit 1

    git restore --staged -- "$README_PATH" 2>/dev/null || true

    if [[ -z "$(git status --porcelain --untracked-files=all -- . ":(exclude)$README_PATH")" ]]; then
        echo "[auto-git] No meaningful changes outside $README_PATH."
        continue
    fi

    git add -A -- . ":(exclude)$README_PATH"
    git restore --staged -- "$README_PATH" 2>/dev/null || true

    if git diff --cached --quiet; then
        echo "[auto-git] No staged changes outside $README_PATH."
        continue
    fi

    COMMIT_MSG="auto: update $(date '+%Y-%m-%d %H:%M:%S')"

    if git commit -m "$COMMIT_MSG"; then
        git push origin "$BRANCH"
        echo "[auto-git] Pushed to origin/$BRANCH"
    else
        echo "[auto-git] Commit failed. Maybe no changes."
    fi
done
