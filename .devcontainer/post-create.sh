#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${HEGE_WS:-/workspaces/hege_gps_navigation}"

set +u
source /opt/ros/humble/setup.bash
set -u

sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro humble

if find "${WORKSPACE}/src" -name package.xml -print -quit 2>/dev/null | grep -q .; then
  rosdep install \
    --from-paths "${WORKSPACE}/src" \
    --ignore-src \
    --rosdistro humble \
    --recursive \
    --yes

  cd "${WORKSPACE}"
  colcon build --symlink-install
else
  echo "No ROS packages exist in ${WORKSPACE}/src yet; skipping colcon build."
fi

grep -qxF 'source /opt/ros/humble/setup.bash' "${HOME}/.bashrc" \
  || echo 'source /opt/ros/humble/setup.bash' >> "${HOME}/.bashrc"

grep -qxF '[ -f "${HEGE_WS}/install/setup.bash" ] && source "${HEGE_WS}/install/setup.bash"' "${HOME}/.bashrc" \
  || echo '[ -f "${HEGE_WS}/install/setup.bash" ] && source "${HEGE_WS}/install/setup.bash"' >> "${HOME}/.bashrc"

echo "Hege environment ready: ROS_DISTRO=${ROS_DISTRO}, workspace=${WORKSPACE}"
