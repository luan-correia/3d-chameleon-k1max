#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)

if command -v git >/dev/null 2>&1 && [ -d "$REPO_DIR/.git" ]; then
    git -C "$REPO_DIR" pull --ff-only
fi

exec "$SCRIPT_DIR/install.sh"
