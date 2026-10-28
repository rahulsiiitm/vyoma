#!/bin/bash
set -euo pipefail

CONTAINER_NAME=${VYOMA_CONTAINER:-vyoma-dev}
WORKSPACE=${VYOMA_WORKSPACE:-/home/drone_user/workspace}
INSTALL=${VYOMA_INSTALL:-/tmp/vyoma-runtime/install}
BUILD_BASE=${VYOMA_BUILD_BASE:-/tmp/vyoma-runtime/build}
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" != true ]; then
  echo "Vyoma container is not running. Start it with ./run_docker.sh first."
  exit 1
fi

# Keep paths and arguments literal by passing them positionally.
if [ "${VYOMA_SKIP_BUILD:-0}" != 1 ]; then
  docker exec "$CONTAINER_NAME" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    cd "$1"
    colcon --log-base /tmp/vyoma-runtime/log build --build-base "$2" --install-base "$3" \
      --symlink-install --packages-select px4_msgs semantic_nav_pkg semantic_vision_pkg rl_navigation_pkg
  ' vyoma-build "$WORKSPACE" "$BUILD_BASE" "$INSTALL"
fi

EXEC_OPTIONS=(-i)
if [ -t 0 ] && [ -t 1 ]; then EXEC_OPTIONS+=(-t); fi
docker exec "${EXEC_OPTIONS[@]}" "$CONTAINER_NAME" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source "$2/setup.bash"
  workspace=$1
  shift 2
  exec python3 "$workspace/scripts/simulation_session.py" --workspace "$workspace" "$@"
' vyoma-session "$WORKSPACE" "$INSTALL" "$@"
