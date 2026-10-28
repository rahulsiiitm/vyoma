#!/bin/bash

# Ensure we are in the workspace
cd /home/drone_user/workspace

echo "Setting up Micro XRCE-DDS Agent..."
AGENT_DIR="Micro-XRCE-DDS-Agent-v2.4.2"
if [ ! -d "$AGENT_DIR" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
fi

cd "$AGENT_DIR"
git fetch --tags
git checkout 57d086216d01ec43121845d385894a25987f8a2c
git submodule update --init --recursive
AGENT_PATCH="/home/drone_user/workspace/patches/micro-xrce-dds-agent-v2.4.2-fastdds.patch"
if git apply --check "$AGENT_PATCH" 2>/dev/null; then
    git apply "$AGENT_PATCH"
fi
mkdir -p build-v2.12.2
cd build-v2.12.2
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

echo "Downloading px4_msgs for ROS 2 workspace..."
cd /home/drone_user/workspace/src
if [ ! -d "px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git
fi
cd px4_msgs
git fetch --tags
git checkout ff7ae284c4b9cb1c39d182e9f1a1343b3817011e

cd /home/drone_user/workspace

echo "---------------------------------------------------"
echo "Micro XRCE-DDS Agent installed successfully!"
echo "To start the bridge, open a new terminal in the container and run:"
echo "MicroXRCEAgent udp4 -p 8888"
echo "---------------------------------------------------"
