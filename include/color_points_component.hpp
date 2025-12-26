#ifndef COLOR_POINTS_COMPONENT_HPP
#define COLOR_POINTS_COMPONENT_HPP

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <omp.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "bag_manager.hpp"

using PointCloud2 = sensor_msgs::msg::PointCloud2;
// using Image = sensor_msgs::msg::Image;
using Image = sensor_msgs::msg::CompressedImage;
using namespace std::chrono_literals;

namespace color_pointscloud
{

struct PointXYZIRCAEDT
{
  PCL_ADD_POINT4D
  std::uint8_t intensity{0U};
  std::uint8_t return_type{0U};
  std::uint16_t channel{0U};
  float azimuth{0.0F};
  float elevation{0.0F};
  float distance{0.0F};
  std::uint32_t time_stamp{0U};
}EIGEN_ALIGN16;

struct PointXYZRGBT
{
  PCL_ADD_POINT4D;
  union
  {
    struct { uint8_t b,g,r,a; }rgba_struct;
    uint32_t rgba;
  };
  std::uint32_t time_stamp{0U};
}EIGEN_ALIGN16;

// 数据结构定义
struct ExtrinsicParams {
    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Matrix4f transformation;
};

struct IntrinsicParams {
    double fx;
    double fy;
    double cx;
    double cy;
    std::vector<double> k;
    Eigen::Matrix3f camera_matrix;
};

struct PointCloudConfig {
    std::string topic;
    std::string frame_id;
    ExtrinsicParams extrinsic;
};

struct ImageConfig {
    std::string topic;
    std::string name;
    IntrinsicParams intrinsic;
    ExtrinsicParams extrinsic;
};

class ColorPointsComponent : public rclcpp::Node
{
public:
    explicit ColorPointsComponent(const rclcpp::NodeOptions & options);
    ~ColorPointsComponent();

private:
    BagManager *bag_manager_;
    size_t gnss_count = 0;
    size_t imu_count = 0;
    size_t points_count = 0;

    std::vector<std::pair<cv::Mat, cv::Mat>> undistort_maps_;
    std::vector<cv::Size> image_size_cache_;
    std::vector<bool> build_map_flag_;

    struct GeneralConfig {
        int image_queue_size;
        double max_time_diff;
        bool use_tf;
        std::string output_topic;
        std::string bag_file_path;
    };

    GeneralConfig general_config_;
    std::vector<PointCloudConfig> pointcloud_configs_;
    std::vector<ImageConfig> image_configs_;

    // ROS2相关
    rclcpp::Publisher<PointCloud2>::SharedPtr colored_cloud_pub_;
    std::vector<std::shared_ptr<message_filters::Subscriber<PointCloud2>>> cloud_subs_;
    std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> image_subs_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bag_control_service_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        PointCloud2, PointCloud2, Image, Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;


    void build_map(const int index);
    bool loadConfig(const std::string& config_file);
    cv::Mat processJPEGImage(const Image::ConstSharedPtr& image_msg, size_t index);
    cv::Mat processRGBAImage(const Image::ConstSharedPtr& image_msg);
    pcl::PointCloud<PointXYZRGBT>::Ptr colorPointCloud(const pcl::PointCloud<PointXYZRGBT>::Ptr& cloud,const std::vector<cv::Mat>& images);
    pcl::PointCloud<PointXYZRGBT>::Ptr colorPointCloud_V1(const pcl::PointCloud<PointXYZRGBT>::Ptr& cloud,const std::vector<cv::Mat>& images);
    void printConfigSummary();
    void initSubscribers();
    void initSynchronizer();
    void initServices();
    void syncCallback(const PointCloud2::ConstSharedPtr& cloud1_msg, 
                  const PointCloud2::ConstSharedPtr& cloud2_msg, 
                  const Image::ConstSharedPtr& image1_msg, 
                  const Image::ConstSharedPtr& image2_msg);
    void handleBagControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
};

} // namespace color_pointscloud

#endif // COLOR_POINTS_COMPONENT_HPP