#!/bin/bash
set -euo pipefail

CONTAINER_ID=$(docker ps -q -f name=vyoma-dev)
if [ -z "$CONTAINER_ID" ]; then
  echo "Vyoma container is not running. Start it with ./run_docker.sh first."
  exit 1
fi

docker exec "$CONTAINER_ID" bash -lc \
  "cd ~/workspace && colcon build --symlink-install --packages-select px4_msgs semantic_nav_pkg semantic_vision_pkg rl_navigation_pkg"

docker exec "$CONTAINER_ID" bash -lc \
  "pkill -x MicroXRCEAgent || true; pkill -x parameter_bridge || true; \
  pkill -x planner_server || true; pkill -x px4 || true; pkill -x gz-server || true; \
  pkill -f '^/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge' || true"

docker exec -d -e PX4_GZ_WORLD=training_world "$CONTAINER_ID" bash -lc \
  "cd ~/workspace/simulation/PX4-Autopilot && make px4_sitl gz_x500_depth"
docker exec -d -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 "$CONTAINER_ID" bash -lc \
  "source /opt/ros/humble/setup.bash && MicroXRCEAgent udp4 -p 8888"
docker exec -d "$CONTAINER_ID" bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge \
  /world/training_world/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /world/training_world/control@ros_gz_interfaces/srv/ControlWorld \
  --ros-args -r /world/training_world/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/image_raw"

sleep 10
docker exec -it "$CONTAINER_ID" bash -lc \
  "source /opt/ros/humble/setup.bash && source ~/workspace/install/setup.bash && \
  ros2 launch semantic_nav_pkg autonomous.launch.py"
