#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [--clean | --cmake-clean-cache] [--extra-colcon-args ...]

Build or clean the s-slam workspace.

Options:
  --clean              Remove build/, install/, and log/, then exit.
  --cmake-clean-cache  Build after clearing CMake package caches.

Environment:
  BUILD_JOBS  Override per-package build parallelism. Defaults to min(half CPU cores, 4).

Examples:
  scripts/build.sh
  scripts/build.sh --clean
  BUILD_JOBS=2 scripts/build.sh
  scripts/build.sh -- --packages-select kiss_matcher_ros
EOF
}

CMAKE_CLEAN_ARGS=()
EXTRA_ARGS=()
CLEAN_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN_ONLY=true
      shift
      ;;
    --cmake-clean-cache)
      CMAKE_CLEAN_ARGS+=(--cmake-clean-cache)
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${BUILD_JOBS:-}" ]]; then
  cpu_count="$(nproc)"
  BUILD_JOBS=$(( (cpu_count + 1) / 2 ))
  if (( BUILD_JOBS > 4 )); then
    BUILD_JOBS=4
  fi
fi

if (( BUILD_JOBS < 1 )); then
  BUILD_JOBS=1
fi

cd "${WORKSPACE_DIR}"

if [[ "${CLEAN_ONLY}" == true ]]; then
  echo "Removing build/, install/, and log/ from ${WORKSPACE_DIR}"
  rm -rf build install log
  exit 0
fi

echo "Building ${WORKSPACE_DIR}"
echo "Package executor=sequential"
echo "Per-package BUILD_JOBS=${BUILD_JOBS}"

export MAKEFLAGS="-j${BUILD_JOBS} -l${BUILD_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"

exec colcon build \
  --executor sequential \
  "${CMAKE_CLEAN_ARGS[@]}" \
  "${EXTRA_ARGS[@]}" \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_C_COMPILER=/usr/bin/cc \
    -DCMAKE_CXX_COMPILER=/usr/bin/c++
