cmake_minimum_required(VERSION 3.16)
project(color_pointscloud)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找OpenMP
find_package(OpenMP REQUIRED)

# 如果支持OpenMP，添加编译选项
if(OpenMP_CXX_FOUND)
  message(STATUS "Found OpenMP: ${OpenMP_CXX_VERSION}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
  message(WARNING "OpenMP not found, performance may be degraded")
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
find_package(ament_index_cpp REQUIRED)

# Find system packages
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
)

# Create library
add_library(color_pointscloud_component SHARED
  src/color_points_component.cpp
)

# 创建普通节点可执行文件
add_executable(color_pointscloud_node
  src/color_points_component.cpp  # 你需要创建这个文件
  src/main.cpp
)

# 链接相同的库
target_include_directories(color_pointscloud_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(color_pointscloud_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_msgs
  std_srvs
  pcl_conversions
  rclcpp_components
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  rosbag2_cpp
  ament_index_cpp
)

target_link_libraries(color_pointscloud_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  Eigen3::Eigen
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(color_pointscloud_node OpenMP::OpenMP_CXX)
endif()

# 安装可执行文件
install(TARGETS color_pointscloud_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)



# Set target properties
target_include_directories(color_pointscloud_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(color_pointscloud_component OpenMP::OpenMP_CXX)
endif()
# Add ROS dependencies
ament_target_dependencies(color_pointscloud_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  pcl_msgs
  std_srvs
  pcl_conversions
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  rosbag2_cpp
  ament_index_cpp
)

# Link system libraries
target_link_libraries(color_pointscloud_component
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  Eigen3::Eigen
)

# Register component
rclcpp_components_register_nodes(color_pointscloud_component
  "color_pointscloud::ColorPointsComponent"
)

# Install library
install(TARGETS color_pointscloud_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install scripts
install(DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script/
  USE_SOURCE_PERMISSIONS
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_components
)

ament_export_libraries(color_pointscloud_component)
ament_export_include_directories(include)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()