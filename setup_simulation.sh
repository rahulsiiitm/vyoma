#!/bin/bash
set -euo pipefail

# Ensure we are in the workspace
cd /home/drone_user/workspace

echo "Installing required Python dependencies for PX4..."
pip3 install --user kconfiglib jinja2 jsonschema future packaging toml numpy pyyaml empy==3.3.4 pyros-genmsg

cd simulation
if [ ! -d "PX4-Autopilot" ]; then
    echo "Cloning PX4-Autopilot with submodules (this may take a few minutes)..."
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
else
    echo "PX4-Autopilot already cloned."
fi

cd PX4-Autopilot
git fetch --tags
git checkout bfb48ad17e1afbad2220207f51cf5fb7e3fc7cd0
git submodule update --init --recursive
PX4_PATCH="/home/drone_user/workspace/patches/px4-lockstep-timedwait.patch"
PX4_PATCH_UPGRADE="/home/drone_user/workspace/patches/px4-lockstep-v1-to-v2.patch"
if git apply --check "$PX4_PATCH" 2>/dev/null; then
    git apply "$PX4_PATCH"
elif git apply --reverse --check "$PX4_PATCH" 2>/dev/null; then
    : # Already applied.
elif git apply --check "$PX4_PATCH_UPGRADE" 2>/dev/null; then
    git apply "$PX4_PATCH_UPGRADE"
    if ! git apply --reverse --check "$PX4_PATCH" 2>/dev/null; then
        git apply --reverse "$PX4_PATCH_UPGRADE"
        echo "PX4 lockstep upgrade did not produce the expected patch." >&2
        exit 1
    fi
else
    echo "PX4 lockstep patch does not match this checkout." >&2
    exit 1
fi

echo "Building PX4 SITL and Gazebo target..."
echo "This will take a while on the first run as it compiles the entire flight stack."
cp /home/drone_user/workspace/simulation/worlds/training_world.world \
  Tools/simulation/gz/worlds/training_world.sdf
export PX4_GZ_WORLD=training_world
# The gz_x500_depth target starts PX4 even with DONT_RUN on this pinned version.
make px4_sitl_default
cmake --build build/px4_sitl_default --target px4_gz_plugins

echo "---------------------------------------------------"
echo "Build complete! To test the simulation, run:"
echo "cd ~/workspace/simulation/PX4-Autopilot && make px4_sitl gz_x500_depth"
echo "---------------------------------------------------"
