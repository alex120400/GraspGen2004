#!/bin/bash

# Base directory on host
BASE_DIR="$(pwd)"  # This should be GraspGen2004

# Paths inside code
CODE_DIR="${BASE_DIR}/code"
MODELS_DIR="${CODE_DIR}/GraspGenModels"
SCENES_DIR="${CODE_DIR}/scenes"

# ROS workspace on host
CATKIN_WS="${BASE_DIR}/catkin_ws"

# Build volume mounts
VOLUME_MOUNTS="-v ${BASE_DIR}:/GraspGen2004"

# Add optional mounts
[ -d "$MODELS_DIR" ] && VOLUME_MOUNTS="$VOLUME_MOUNTS -v ${MODELS_DIR}:/GraspGen2004/code/GraspGenModels"
# [ -d "$SCENES_DIR" ] && VOLUME_MOUNTS="$VOLUME_MOUNTS -v ${SCENES_DIR}:/GraspGen2004/code/scenes"
[ -d "$CATKIN_WS" ] && VOLUME_MOUNTS="$VOLUME_MOUNTS -v ${CATKIN_WS}:/GraspGen2004/catkin_ws"

echo "Starting Docker container with:"
echo "  Base directory: $BASE_DIR -> /GraspGen2004"
echo "  GPU enabled: yes"
echo ""

# Allow X11 forwarding for visualization
xhost +local:root

HOST_LAN_IP=$(hostname -I | awk '{print $1}')
ROBOT_IP=10.0.0.143   # Sasha's IP

docker run \
  --name graspgen_ros1_container \
  --privileged \
  --gpus all \
  --ipc=host \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  -e NVIDIA_DISABLE_REQUIRE=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_IP=${HOST_LAN_IP} \
  -e ROS_MASTER_URI="http://${ROBOT_IP}:11311" \
  --device /dev/dri \
  --net host \
  -e DISPLAY \
  -it \
  $VOLUME_MOUNTS \
  graspgen:cuda121 \
  /bin/bash -c "cd /GraspGen2004 && pip install -e ./code && bash"

xhost -local:root
