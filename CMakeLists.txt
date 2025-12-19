cmake_minimum_required(VERSION 3.8)
project(color_pointscloud)

# ---------- compiler ----------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++14 (PCL 1.10+ 需要)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- find packages ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# ---------- PCL/ROS 宏 ----------
add_definitions(${PCL_DEFINITIONS})
# 如果用 pcl_ros 的 transform 相关函数，再打开下面两行
# find_package(pcl_ros REQUIRED)
# list(APPEND PCL_ROS_DEPS pcl_ros)

# ---------- build ----------
# add_library(color_pointscloud_node SHARED
#   src/color_points_node.cpp
# )
add_executable(color_pointscloud_node
  src/color_points_node.cpp)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${rosbag2_cpp_INCLUDE_DIRS}
)

# ROS 依赖
ament_target_dependencies(color_pointscloud_node
  rclcpp
  # rclcpp_components
  std_srvs
  sensor_msgs
  std_msgs
  pcl_msgs
  pcl_conversions
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  rosbag2_cpp
)


# 第三方库
target_link_libraries(color_pointscloud_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${rosbag2_cpp_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
  Eigen3::Eigen
)

# 头文件目录
target_include_directories(color_pointscloud_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)


install(TARGETS color_pointscloud_node
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()