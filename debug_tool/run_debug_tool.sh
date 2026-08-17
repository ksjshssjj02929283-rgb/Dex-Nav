#!/usr/bin/env bash
set -euo pipefail

tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(dirname -- "${tool_dir}")"
cd -- "${repo_dir}"
exec python3 -m debug_tool.run "$@"
