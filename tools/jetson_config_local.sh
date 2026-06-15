#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "error: not inside a git worktree: $ROOT_DIR" >&2
  exit 1
fi

if [[ -f config.yaml && ! -f config.local.yaml ]]; then
  cp config.yaml config.local.yaml
  echo "created config.local.yaml from current config.yaml"
elif [[ -f config.local.yaml ]]; then
  echo "kept existing config.local.yaml"
else
  echo "error: config.yaml does not exist" >&2
  exit 1
fi

git checkout origin/main -- config.yaml
git update-index --skip-worktree config.yaml

python3 - <<'PY'
import yaml

for path in ("config.yaml", "config.local.yaml"):
    with open(path, "r", encoding="utf-8") as f:
        yaml.safe_load(f)
print("config.yaml and config.local.yaml are valid YAML")
PY

echo "done"
echo "future local Jetson settings should go in config.local.yaml"
echo "normal pulls can use: git pull"
