# Color PointsCloud ROS2 Node

这是一个ROS2 Humble节点，用于将点云数据与图像数据进行着色融合。

## 功能

- 订阅点云消息 (`sensor_msgs/PointCloud2`)
- 订阅图像消息 (`sensor_msgs/Image`)
- 使用TF进行坐标变换
- 发布带颜色的点云消息

## 使用方法

### 构建

```bash
cd /home/pix/code/ros2
colcon build --packages-select color_pointscloud
```

### 运行

```bash
# 方法1: 使用组件容器
ros2 run rclcpp_components component_container

# 然后加载节点
ros2 component load /ComponentManager color_pointscloud color_pointscloud::ColorPointsNode

# 方法2: 使用启动文件
ros2 launch color_pointscloud color_points.launch.py
```

### 参数配置

可以通过参数文件或命令行参数配置节点：

```bash
ros2 run color_pointscloud color_points_node --ros-args -p pointcloud_topic:=/velodyne_points -p image_topic:=/camera/image_raw
```

## 参数说明

- `pointcloud_topic`: 点云订阅话题 (默认: `/pointcloud`)
- `image_topic`: 图像订阅话题 (默认: `/image`)
- `output_topic`: 输出话题 (默认: `/colored_pointcloud`)
- `target_frame`: 目标坐标系 (默认: `base_link`)

## 依赖

- ROS2 Humble
- PCL (Point Cloud Library)
- OpenCV
- TF2