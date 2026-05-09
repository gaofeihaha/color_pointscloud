#!/bin/bash

CONTAINER_NAME="robobus_mapping_container"

# 1. 检查容器是否运行
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME is not running."
    exit 0
fi

echo "Stopping running ROS 2 nodes safely inside $CONTAINER_NAME..."

# 在容器内安全结束ROS节点 (发生 SIGINT)
# 这里假设想要关掉所有 ros2 进程（也可以指定名字，比如 killall test_node）
docker exec $CONTAINER_NAME bash -c "killall -SIGINT ros2"
# 等待给节点保存日志和清理资源的机会
sleep 2

# 强制结束由于一直挂起没关掉的节点
docker exec $CONTAINER_NAME bash -c "killall -SIGTERM ros2 2>/dev/null"

echo "Stopping container $CONTAINER_NAME..."
# 停止容器
docker stop $CONTAINER_NAME

if [ $? -eq 0 ]; then
    echo "Container $CONTAINER_NAME stopped successfully."
else
    echo "Failed to stop container."
    exit 1
fi
