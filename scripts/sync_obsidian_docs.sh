#!/bin/bash
set -e

SRC="$HOME/Documents/Obsidian/2026/1학기/UROP/Project"
DST="$HOME/turtlebot3_ws/src/docs"

mkdir -p "$DST"

rsync -av --delete \
  --include='*/' \
  --include='*.md' \
  --exclude='*' \
  "$SRC/" "$DST/"

echo "Obsidian markdown synced to turtlebot3_ws/src/docs"
