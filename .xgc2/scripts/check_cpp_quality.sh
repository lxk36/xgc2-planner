#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if ! command -v clang-format >/dev/null 2>&1; then
  echo "clang-format is not installed" >&2
  exit 1
fi

mapfile -t files < <(
  find "${REPO_ROOT}/gcopter" \
    -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
    | sort
)

if [[ "${#files[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

for file in "${files[@]}"; do
  clang-format --dry-run --Werror "${file}"
done

echo "C++ quality check passed"
