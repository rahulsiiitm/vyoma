#!/bin/bash

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

echo "Building PX4 SITL and Gazebo target..."
echo "This will take a while on the first run as it compiles the entire flight stack."
# DONT_RUN=1 ensures it just builds and doesn't immediately launch the simulator
cp /home/drone_user/workspace/simulation/worlds/training_world.world \
  Tools/simulation/gz/worlds/training_world.sdf
export PX4_GZ_WORLD=training_world
DONT_RUN=1 make px4_sitl gz_x500_depth

echo "---------------------------------------------------"
echo "Build complete! To test the simulation, run:"
echo "cd ~/workspace/simulation/PX4-Autopilot && make px4_sitl gz_x500_depth"
echo "---------------------------------------------------"
