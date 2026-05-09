#!/bin/bash

CONTAINER_NAME="robobus_mapping_container"
IMAGE_NAME="robobus_mapping_web:latest"

# 替换成你真正在 docker 内部需要启动的 ROS 2 命令，比如:
# ROS2_COMMAND="source /opt/ros/\$ROS_DISTRO/setup.bash && source /root/webserver_ws/install/setup.bash && ros2 launch your_package your_launch.py"
ROS2_COMMAND="source /opt/ros/\$ROS_DISTRO/setup.bash && source /root/webserver_ws/install/setup.bash &&  ros2 run mqtt_node mqtt_node"

# 1. 检查是否已运行
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME is already running."
    echo "Starting ROS 2 node in the running container..."
    # 使用 -d 以分离模式（后台）在容器内运行程序
    docker exec -d $CONTAINER_NAME bash -c "$ROS2_COMMAND"
    exit 0
fi

# 2. 检查是否存在停止的容器并删除
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Removing stopped container $CONTAINER_NAME..."
    docker rm $CONTAINER_NAME
fi

# GPU 支持检测
GPU_OPT=""
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected, enabling runtime support."
    GPU_OPT="--gpus all"
fi

# X11 图形界面支持
xhost +local:root

# 挂载工作空间 (请根据需要修改宿主机和容器内的挂载路径)
HOST_WORKSPACE_DIR="$HOME/pix/robobus_mapping_web"
CONTAINER_WORKSPACE_DIR="/root/webserver_ws"

echo "Starting container..."
echo "Host Workspace: $HOST_WORKSPACE_DIR"
echo "Container Workspace: $CONTAINER_WORKSPACE_DIR"

# 3. 运行容器
docker run -it -d \
    --name $CONTAINER_NAME \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOST_WORKSPACE_DIR":"$CONTAINER_WORKSPACE_DIR" \
    $GPU_OPT \
    "$IMAGE_NAME"

if [ $? -eq 0 ]; then
    echo "Container started successfully."
    echo "Starting ROS 2 node inside the new container..."
    docker exec -d $CONTAINER_NAME bash -c "$ROS2_COMMAND"
else
    echo "Failed to start container."
    exit 1
fi
