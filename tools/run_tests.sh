#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if [[ -x ".venv/bin/python" ]]; then
  python_bin=".venv/bin/python"
else
  python_bin="$(command -v python3)"
fi

export PYTHONPATH="$repo_root${PYTHONPATH:+:$PYTHONPATH}"
export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-dummy}"

test_files=()
while IFS= read -r test_file; do
  test_files+=("$test_file")
done < <(find tests -maxdepth 1 -type f -name 'test_*.py' | sort)
if [[ ${#test_files[@]} -eq 0 ]]; then
  echo "No regression scripts found under tests/" >&2
  exit 1
fi

for test_file in "${test_files[@]}"; do
  echo "==> $test_file"
  "$python_bin" "$test_file"
done

echo "All regression scripts passed (${#test_files[@]} files)."
