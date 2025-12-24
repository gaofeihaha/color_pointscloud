#include "rclcpp/rclcpp.hpp"
#include "color_points_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建节点选项
  rclcpp::NodeOptions options;
  
  // 创建组件实例
  auto node = std::make_shared<color_pointscloud::ColorPointsComponent>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}