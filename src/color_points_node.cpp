#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <omp.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#include <std_srvs/srv/set_bool.hpp> // 添加服务消息头文件

#include "bag_manager.hpp"

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::CompressedImage;
using namespace std::chrono_literals;

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
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRCAEDT, 
      (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
      (std::uint8_t, return_type, return_type)(std::uint16_t, channel, channel)
      (float, azimuth, azimuth)(float, elevation, elevation)(float, distance, distance)
      (std::uint32_t, time_stamp, time_stamp)
)

struct PointXYZRGBRT
{
  PCL_ADD_POINT4D;                     // x,y,z + padding
  union
  {
    struct { uint8_t b,g,r,a; }rgba_struct;       // BGRA 顺序，PCL 惯例
    uint32_t rgba;
  };
  std::uint16_t channel{0U};
  std::uint32_t time_stamp{0U};
  
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBRT, 
      (float, x, x)(float, y, y)(float, z, z)(uint32_t, rgba, rgba)
      (std::uint16_t, channel, channel)
      (std::uint32_t, time_stamp, time_stamp)
)

// 数据结构定义
struct ExtrinsicParams {
    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Matrix4f transformation; // 完整的变换矩阵
};

struct IntrinsicParams {
    double fx;
    double fy;
    double cx;
    double cy;
    std::vector<double> k;
    Eigen::Matrix3f camera_matrix; // 内参矩阵
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

class CloudImageSyncNode : public rclcpp::Node
{
public:
    CloudImageSyncNode() : Node("cloud_image_sync_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // 从参数获取配置文件路径
        this->declare_parameter<std::string>("config_file", "");
        std::string config_file = this->get_parameter("config_file").as_string();
        
        if (config_file.empty()) {
            // 默认路径
            config_file = "src/color_pointscloud/config/config.yaml";
            RCLCPP_WARN(this->get_logger(), "Config file not specified, using default: %s", config_file.c_str());
        }
        
        // 加载配置
        if (!loadConfig(config_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config file!");
            rclcpp::shutdown();
            return;
        }

        undistort_maps_.resize(2);
        image_size_cache_.resize(2);
        
        // 初始化订阅者
        initSubscribers();
        
        // 初始化同步器
        initSynchronizer();
        
        // 初始化服务
        initServices();
        
        // 初始化发布者
        colored_cloud_pub_ = this->create_publisher<PointCloud2>(
            general_config_.output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "CloudImageSyncNode initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", general_config_.output_topic.c_str());

        bag_manager_ = new BagManager("/home/pix/data/colcor_points/test_bag", 100000);

    }
    ~CloudImageSyncNode()
    {
        std::cout<<"~CloudImageSyncNode()"<<std::endl;
        delete bag_manager_;
    };

private:
    BagManager *bag_manager_;

private:
    std::vector<std::pair<cv::Mat, cv::Mat>> undistort_maps_;
    
    // 缓存图像尺寸，用于检查是否需要重新计算映射表
    std::vector<cv::Size> image_size_cache_;

    struct GeneralConfig {
        int image_queue_size;
        double max_time_diff;
        bool use_tf;
        std::string output_topic;
    };
    
    bool loadConfig(const std::string& config_file)
    {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            
            // 加载通用参数
            auto general = config["general"];
            general_config_.image_queue_size = general["image_queue_size"].as<int>();
            general_config_.max_time_diff = general["max_time_diff"].as<double>();
            general_config_.use_tf = general["use_tf"].as<bool>();
            general_config_.output_topic = general["output_topic"].as<std::string>();
            
            // 加载点云配置
            pointcloud_configs_.clear();
            auto pc_topics = config["pointcloud_topics"];
            for (const auto& pc_config : pc_topics) {
                PointCloudConfig config;
                config.topic = pc_config["topic"].as<std::string>();
                config.frame_id = pc_config["frame_id"].as<std::string>();
                
                // 解析外参
                auto ext = pc_config["extrinsic_params"];
                auto rot = ext["rotation"].as<std::vector<double>>();
                auto trans = ext["translation"].as<std::vector<double>>();
                
                // 注意：YAML中的旋转矩阵是行优先存储，需要转换为Eigen矩阵
                Eigen::Matrix3f rotation_matrix;
                rotation_matrix << rot[0], rot[1], rot[2],
                                  rot[3], rot[4], rot[5],
                                  rot[6], rot[7], rot[8];
                
                config.extrinsic.rotation = rotation_matrix;
                config.extrinsic.translation = Eigen::Vector3f(trans[0], trans[1], trans[2]);
                
                // 构建变换矩阵
                config.extrinsic.transformation.setIdentity();
                config.extrinsic.transformation.block<3,3>(0,0) = rotation_matrix;
                config.extrinsic.transformation.block<3,1>(0,3) = config.extrinsic.translation;
                
                pointcloud_configs_.push_back(config);
            }
            
            // 加载图像配置
            image_configs_.clear();
            auto img_topics = config["image_topics"];
            for (const auto& img_config : img_topics) {
                ImageConfig config;
                config.topic = img_config["topic"].as<std::string>();
                config.name = img_config["name"].as<std::string>();
                
                // 解析内参
                auto intr = img_config["intrinsic_params"];
                config.intrinsic.fx = intr["fx"].as<double>();
                config.intrinsic.fy = intr["fy"].as<double>();
                config.intrinsic.cx = intr["cx"].as<double>();
                config.intrinsic.cy = intr["cy"].as<double>();
                config.intrinsic.k = intr["k"].as<std::vector<double>>();
                
                // 构建内参矩阵
                config.intrinsic.camera_matrix.setZero();
                config.intrinsic.camera_matrix(0,0) = config.intrinsic.fx;
                config.intrinsic.camera_matrix(1,1) = config.intrinsic.fy;
                config.intrinsic.camera_matrix(0,2) = config.intrinsic.cx;
                config.intrinsic.camera_matrix(1,2) = config.intrinsic.cy;
                config.intrinsic.camera_matrix(2,2) = 1.0;
                
                // 解析外参
                auto ext = img_config["extrinsic_params"];
                auto rot = ext["rotation"].as<std::vector<double>>();
                auto trans = ext["translation"].as<std::vector<double>>();
                
                Eigen::Matrix3f rotation_matrix;
                rotation_matrix << rot[0], rot[1], rot[2],
                                  rot[3], rot[4], rot[5],
                                  rot[6], rot[7], rot[8];
                
                config.extrinsic.rotation = rotation_matrix;
                config.extrinsic.translation = Eigen::Vector3f(trans[0], trans[1], trans[2]);
                
                // 构建变换矩阵
                config.extrinsic.transformation.setIdentity();
                config.extrinsic.transformation.block<3,3>(0,0) = rotation_matrix;
                config.extrinsic.transformation.block<3,1>(0,3) = config.extrinsic.translation;
                
                image_configs_.push_back(config);
            }
            
            printConfigSummary();
            return true;
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading config: %s", e.what());
            return false;
        }
    }
    
    void printConfigSummary()
    {
        RCLCPP_INFO(this->get_logger(), "=== Configuration Summary ===");
        RCLCPP_INFO(this->get_logger(), "General:");
        RCLCPP_INFO(this->get_logger(), "  Queue size: %d", general_config_.image_queue_size);
        RCLCPP_INFO(this->get_logger(), "  Max time diff: %.3f seconds", general_config_.max_time_diff);
        RCLCPP_INFO(this->get_logger(), "  Use TF: %s", general_config_.use_tf ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", general_config_.output_topic.c_str());
        
        RCLCPP_INFO(this->get_logger(), "\nPointCloud Topics (%zu):", pointcloud_configs_.size());
        for (size_t i = 0; i < pointcloud_configs_.size(); ++i) {
            const auto& config = pointcloud_configs_[i];
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s (frame: %s)", 
                       i, config.topic.c_str(), config.frame_id.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "\nImage Topics (%zu):", image_configs_.size());
        for (size_t i = 0; i < image_configs_.size(); ++i) {
            const auto& config = image_configs_[i];
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s (name: %s)", 
                       i, config.topic.c_str(), config.name.c_str());
            RCLCPP_INFO(this->get_logger(), "      Intrinsic: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                       config.intrinsic.fx, config.intrinsic.fy,
                       config.intrinsic.cx, config.intrinsic.cy);
            RCLCPP_INFO(this->get_logger(), "      Extrinsic: rot=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], trans=[%.2f, %.2f, %.2f]",
                       config.extrinsic.rotation(0,0), config.extrinsic.rotation(0,1), config.extrinsic.rotation(0,2),
                       config.extrinsic.rotation(1,0), config.extrinsic.rotation(1,1), config.extrinsic.rotation(1,2),
                       config.extrinsic.rotation(2,0), config.extrinsic.rotation(2,1), config.extrinsic.rotation(2,2),
                       config.extrinsic.translation(0), config.extrinsic.translation(1), config.extrinsic.translation(2));
        }
        RCLCPP_INFO(this->get_logger(), "==========================");
    }
    void initServices()
    {
        // 创建bag记录控制服务
        bag_control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "bag_control",
            std::bind(&CloudImageSyncNode::handleBagControl, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Services initialized: bag_control");
    }
    
    void initSubscribers()
    {
        
        // 创建QoS配置
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 100));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        
        // 初始化点云订阅者
        cloud_subs_.clear();
        for (const auto& config : pointcloud_configs_) {
            auto sub = std::make_shared<message_filters::Subscriber<PointCloud2>>(
                this, config.topic, qos.get_rmw_qos_profile());
            cloud_subs_.push_back(sub);
            RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud: %s", config.topic.c_str());
        }
        
        // 初始化图像订阅者
        image_subs_.clear();
        for (const auto& config : image_configs_) {
            auto sub = std::make_shared<message_filters::Subscriber<Image>>(
                this, config.topic, qos.get_rmw_qos_profile());
            image_subs_.push_back(sub);
            RCLCPP_INFO(this->get_logger(), "Subscribed to image: %s", config.topic.c_str());
        }

        static int gnss_count = 0;
        static int imu_count = 0;



        gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/fix", 100,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
            {
                gnss_count++;
                std::cout << "gnss count: " << gnss_count << std::endl;
            // std::cout << "gnss time: " << msg->header.stamp.sec << "-" << msg->header.stamp.nanosec << std::endl;
                bag_manager_->addMessage(*msg, "/sensing/gnss/fix", msg->header.stamp);
            });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/imu/imu_data", 100,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
            // std::cout << "imu time: " << msg->header.stamp.sec << "-" << msg->header.stamp.nanosec << std::endl;
            std::cout << "imu count: " << imu_count << std::endl;
                imu_count++;
                bag_manager_->addMessage(*msg, "/sensing/imu/imu_data", msg->header.stamp);
            });
    }
    
    void initSynchronizer()
    {
        if (cloud_subs_.size() != 2 || image_subs_.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Expected 2 pointclouds and 2 images, got %zu and %zu",
                        cloud_subs_.size(), image_subs_.size());
            return;
        }
        
        // 使用ApproximateTime同步策略
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            PointCloud2, PointCloud2, Image, Image>;
        
        // 创建同步器
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(general_config_.image_queue_size), 
            *cloud_subs_[0], *cloud_subs_[1], *image_subs_[0], *image_subs_[1]);
        
        // 设置最大时间间隔
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(general_config_.max_time_diff));
        
        // 注册回调函数
        sync_->registerCallback(
            std::bind(&CloudImageSyncNode::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3, std::placeholders::_4));
        
        RCLCPP_INFO(this->get_logger(), "Synchronizer initialized with queue size %d", 
                   general_config_.image_queue_size);
    }
    
    void syncCallback(const PointCloud2::ConstSharedPtr& cloud1_msg, const PointCloud2::ConstSharedPtr& cloud2_msg, const Image::ConstSharedPtr& image1_msg, const Image::ConstSharedPtr& image2_msg)
    {
        static int points_count = 0;
        points_count++;
        std::cout << "points count: " << points_count << std::endl;

        auto msg_header = cloud1_msg->header;
        msg_header.frame_id = "base_link";

        // // 将时间转换为rclcpp::Time类型以便进行运算
        // rclcpp::Time cloud1_time(cloud1_msg->header.stamp);
        // rclcpp::Time cloud2_time(cloud2_msg->header.stamp);
        // rclcpp::Time image1_time(image1_msg->header.stamp);
        // rclcpp::Time image2_time(image2_msg->header.stamp);
        
        // // 计算最大时间差
        // double max_diff = 0.0;
        // max_diff = std::max(max_diff, std::abs((cloud1_time - image1_time).seconds()));
        // max_diff = std::max(max_diff, std::abs((cloud2_time - image2_time).seconds()));
        // max_diff = std::max(max_diff, std::abs((cloud1_time - cloud2_time).seconds()));
        // max_diff = std::max(max_diff, std::abs((image1_time - image2_time).seconds()));

        // RCLCPP_INFO(this->get_logger(), "Received synchronized messages (max time diff: %.6fs)", max_diff);
        
        // if (max_diff > general_config_.max_time_diff) {
        //     RCLCPP_WARN(this->get_logger(), "Time difference too large: %.6f > %.3f seconds", 
        //             max_diff, general_config_.max_time_diff);
        //     return;
        // }


        pcl::PointCloud<PointXYZIRCAEDT>::Ptr tmpRobosenseCloudIn_front(new pcl::PointCloud<PointXYZIRCAEDT>());
        pcl::PointCloud<PointXYZIRCAEDT>::Ptr tmpRobosenseCloudIn_rear(new pcl::PointCloud<PointXYZIRCAEDT>());
        pcl::PointCloud<PointXYZRGBRT>::Ptr cloud_conc(new pcl::PointCloud<PointXYZRGBRT>());

        PointCloudConfig config_front = pointcloud_configs_[0];
        PointCloudConfig config_rear = pointcloud_configs_[1];

        std::uint32_t cloud1_time_ns = cloud1_msg->header.stamp.sec * 1e9 + cloud1_msg->header.stamp.nanosec;
        std::uint32_t cloud2_time_ns = cloud2_msg->header.stamp.sec * 1e9 + cloud2_msg->header.stamp.nanosec;
        std::uint32_t cloud1_diff_ns = 0;
        std::uint32_t cloud2_diff_ns = 0;

        if(cloud1_time_ns < cloud2_time_ns) //点云时间dt调整
        {
            msg_header.stamp = cloud1_msg->header.stamp;
            cloud2_diff_ns = cloud2_time_ns - cloud1_time_ns;
        }
        else
        {
            msg_header.stamp = cloud2_msg->header.stamp;
            cloud1_diff_ns = cloud1_time_ns - cloud2_time_ns;
        }


        auto rosmsg_start = std::chrono::high_resolution_clock::now();

        #pragma omp parallel sections
        {
            #pragma omp section
            {
                sensor_msgs::msg::PointCloud2 msg = std::move(*cloud1_msg);
                pcl::fromROSMsg(msg, *tmpRobosenseCloudIn_front);
            }
            
            #pragma omp section
            {
                sensor_msgs::msg::PointCloud2 msg = std::move(*cloud2_msg);
                pcl::fromROSMsg(msg, *tmpRobosenseCloudIn_rear);
            }
        }

        size_t size_lidar = tmpRobosenseCloudIn_front->size() + tmpRobosenseCloudIn_rear->size();
        cloud_conc->resize(size_lidar);

        auto points_start = std::chrono::high_resolution_clock::now();


        // 并行处理前部点云转换
        #pragma omp parallel for
        for (size_t i = 0; i < tmpRobosenseCloudIn_front->size(); i++) 
        {
            auto &src = tmpRobosenseCloudIn_front->points[i];
            auto &dst = cloud_conc->points[i];
            Eigen::Vector3f point_vec(src.x, src.y, src.z);
            Eigen::Vector3f transformed_vec = config_front.extrinsic.rotation * point_vec + config_front.extrinsic.translation;
            dst.x = transformed_vec.x();
            dst.y = transformed_vec.y();
            dst.z = transformed_vec.z();
            dst.channel = src.channel;
            dst.time_stamp = src.time_stamp + cloud1_diff_ns;
        }
        
        size_t size_front = tmpRobosenseCloudIn_front->size();
        
        // 并行处理后部点云转换
        #pragma omp parallel for
        for (size_t i = 0; i < tmpRobosenseCloudIn_rear->size(); i++) {
            auto &src = tmpRobosenseCloudIn_rear->points[i];
            auto &dst = cloud_conc->points[i+size_front];
            Eigen::Vector3f point_vec(src.x, src.y, src.z);
            Eigen::Vector3f transformed_vec = config_rear.extrinsic.rotation * point_vec + config_rear.extrinsic.translation;
            dst.x = transformed_vec.x();
            dst.y = transformed_vec.y();
            dst.z = transformed_vec.z();
            dst.channel = src.channel;
            dst.time_stamp = src.time_stamp + cloud2_diff_ns;
        }
        auto undistort_start = std::chrono::high_resolution_clock::now();

        
        // 处理图像
        std::vector<cv::Mat> images;
        images.resize(2);
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                images.at(0) = processImage(image1_msg, 0);
            }
            
            #pragma omp section
            {
                images.at(1) = processImage(image2_msg, 1);
            }
        }

        // cv::imshow("Image 0", images[0]);
        // cv::imshow("Image 1", images[1]);
        // cv::waitKey(1);
        auto color_start = std::chrono::high_resolution_clock::now();
        
        // 执行点云上色
        auto colored_cloud = colorPointCloud(cloud_conc, images);

        //转换为PointCloud2
        sensor_msgs::msg::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
        colored_cloud_msg.header = msg_header;
        
        // 写入Bag
        std::cout << "lidar time: " << msg_header.stamp.sec << "-" << msg_header.stamp.nanosec << std::endl;
        bag_manager_->addMessage(
            colored_cloud_msg,
            "/sensing/lidar/points_rgb",
            msg_header.stamp
        );

        auto color_end = std::chrono::high_resolution_clock::now();

        // publishColoredCloud(colored_cloud);

        // std::cout << "ros time: " << std::chrono::duration_cast<std::chrono::milliseconds>(points_start - rosmsg_start).count() << " ms" << std::endl;
        // std::cout << "points time: " << std::chrono::duration_cast<std::chrono::milliseconds>(undistort_start - points_start).count() << " ms" << std::endl;
        // std::cout << "undistort time: " << std::chrono::duration_cast<std::chrono::milliseconds>(color_start - undistort_start).count() << " ms" << std::endl;
        // std::cout << "color time: " << std::chrono::duration_cast<std::chrono::milliseconds>(color_end - color_start).count() << " ms" << std::endl;

    }

    // Bag记录控制服务回调
    void handleBagControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::string command = request->data ? "start" : "stop";
        RCLCPP_INFO(this->get_logger(), "Received bag control command: %s", command.c_str());
        
        bool bag_recording_enabled_ = request->data;
        
        if (bag_recording_enabled_) {
            response->message = "Bag recording started successfully";
            bag_manager_->start_record();
            RCLCPP_INFO(this->get_logger(), "Bag recording enabled");
        } else {
            response->message = "Bag recording stopped successfully";
            bag_manager_->stop_record();
            RCLCPP_INFO(this->get_logger(), "Bag recording disabled");
        }
        
        response->success = true;
    }
    
    cv::Mat processImage(const Image::ConstSharedPtr& image_msg, size_t index)
    {
        cv::Mat image;
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image;

            // 去畸变
            IntrinsicParams intr_params = image_configs_[index].intrinsic;
            
            // 检查图像和畸变系数有效性
            if (image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Image %zu is empty", index);
                return cv::Mat();
            }
            
            // 如果没有畸变系数，返回原图
            if (intr_params.k.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "No distortion coefficients for image %zu, returning original", index);
                return image;
            }
            
            // 创建OpenCV相机矩阵
            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
                intr_params.fx, 0, intr_params.cx,
                0, intr_params.fy, intr_params.cy,
                0, 0, 1);
            
            // 将畸变系数转换为cv::Mat
            cv::Mat dist_coeffs;
            if (intr_params.k.size() == 8) {
                // 8参数畸变模型 [k1, k2, p1, p2, k3, k4, k5, k6]
                dist_coeffs = cv::Mat_<double>(1, 8);
                for (size_t i = 0; i < 8; ++i) {
                    dist_coeffs.at<double>(0, i) = intr_params.k[i];
                }
            } else if (intr_params.k.size() >= 5) {
                // 至少需要5个参数 [k1, k2, p1, p2, k3]
                dist_coeffs = cv::Mat_<double>(1, 5);
                for (size_t i = 0; i < 5 && i < intr_params.k.size(); ++i) {
                    dist_coeffs.at<double>(0, i) = intr_params.k[i];
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid distortion coefficients size %zu for image %zu", 
                        intr_params.k.size(), index);
                return image;
            }
            
            // 使用映射表方法进行去畸变（保持内参不变）
            cv::Mat undistorted;
            
            // 检查是否需要初始化映射表
            if (undistort_maps_[index].first.empty() || undistort_maps_[index].second.empty() ||
                image_size_cache_[index] != image.size()) {
                
                // 初始化映射表（使用原始内参保持内参不变）
                cv::Mat new_camera_matrix = camera_matrix.clone();
                cv::Mat map1, map2;
                
                cv::initUndistortRectifyMap(
                    camera_matrix,        // 原始相机内参
                    dist_coeffs,          // 畸变系数
                    cv::Mat(),            // 无旋转矩阵
                    new_camera_matrix,    // 使用相同的内参矩阵（保持内参不变）
                    image.size(),         // 输出尺寸与输入相同
                    CV_32FC1,             // 映射表数据类型
                    map1, map2           // 输出的映射表
                );
                
                // 缓存映射表和图像尺寸
                undistort_maps_[index] = std::make_pair(map1, map2);
                image_size_cache_[index] = image.size();
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Initialized undistort maps for image %zu with size %dx%d", 
                    index, image.cols, image.rows);
            }
            
            // 应用去畸变映射
            cv::remap(image, undistorted, 
                    undistort_maps_[index].first, 
                    undistort_maps_[index].second,
                    cv::INTER_LINEAR, 
                    cv::BORDER_CONSTANT);
            
            image = undistorted;

            RCLCPP_DEBUG(this->get_logger(), 
                "Processed image %zu: %dx%d, Intrinsic unchanged: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                index, image.cols, image.rows,
                intr_params.fx, intr_params.fy, intr_params.cx, intr_params.cy);
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error processing image %zu: %s", index, e.what());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error processing image %zu: %s", index, e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image %zu: %s", index, e.what());
        }
        return image;
    }
    
    pcl::PointCloud<PointXYZRGBRT>::Ptr colorPointCloud(
    const pcl::PointCloud<PointXYZRGBRT>::Ptr& cloud,
    const std::vector<cv::Mat>& images)
    {
        auto colored_cloud = pcl::PointCloud<PointXYZRGBRT>::Ptr(new pcl::PointCloud<PointXYZRGBRT>());
        
        // 预分配内存以提高性能
        colored_cloud->reserve(cloud->size());
        
        // 提前提取配置信息到局部变量，避免循环中频繁访问
        std::vector<bool> image_valid(images.size());
        std::vector<const IntrinsicParams*> intrinsics_ptrs(images.size(), nullptr);
        std::vector<const ExtrinsicParams*> extrinsics_ptrs(images.size(), nullptr);
        
        for (size_t j = 0; j < images.size(); ++j) {
            image_valid[j] = !images[j].empty();
            if (j < image_configs_.size()) {
                intrinsics_ptrs[j] = &image_configs_[j].intrinsic;
                extrinsics_ptrs[j] = &image_configs_[j].extrinsic;
            }
        }
        
        // 获取图像尺寸用于边界检查
        std::vector<cv::Size> image_sizes(images.size());
        for (size_t j = 0; j < images.size(); ++j) {
            image_sizes[j] = images[j].size();
        }
        
        // 使用OpenMP并行处理所有点
        #pragma omp parallel
        {
            // 每个线程有自己的局部点云，避免锁竞争
            pcl::PointCloud<PointXYZRGBRT>::Ptr local_cloud(new pcl::PointCloud<PointXYZRGBRT>());
            local_cloud->reserve(cloud->size() / omp_get_num_threads() + 1);
            
            #pragma omp for nowait schedule(dynamic, 100)  // 动态调度，块大小为100
            for (size_t i = 0; i < cloud->size(); ++i) 
            {
                const auto& point = cloud->points[i];
                bool point_colored = false;
                
                // 为每个点寻找对应的图像颜色
                for (size_t j = 0; j < images.size() && !point_colored; ++j) 
                {
                    if (!image_valid[j]) continue;
                    
                    // 计算点在相机坐标系中的位置
                    Eigen::Vector3f point_camera;
                    if (general_config_.use_tf) {
                        // 使用TF变换（需要实现TF查询）
                        // point_camera = transformPointWithTF(point, i, j);
                    } else {
                        // 使用配置文件中的外参
                        if (extrinsics_ptrs[j] != nullptr) {
                            const auto& transformation = extrinsics_ptrs[j]->transformation;
                            Eigen::Vector3f point_eigen(point.x, point.y, point.z);
                            point_camera = transformation.block<3,3>(0,0) * point_eigen + 
                                        transformation.block<3,1>(0,3);
                        }
                    }
                    
                    // 如果点在相机前方
                    if (point_camera.z() > 0.1f) 
                    {
                        // 投影到图像平面
                        if (intrinsics_ptrs[j] != nullptr) {
                            const auto& intrinsic = *intrinsics_ptrs[j];
                            
                            // 优化：使用提前计算好的逆深度
                            float inv_z = 1.0f / point_camera.z();
                            float u_f = intrinsic.fx * point_camera.x() * inv_z + intrinsic.cx;
                            float v_f = intrinsic.fy * point_camera.y() * inv_z + intrinsic.cy;
                            
                            // 检查点是否在图像范围内
                            int u = static_cast<int>(u_f);
                            int v = static_cast<int>(v_f);
                            
                            if (u >= 0 && u < image_sizes[j].width && 
                                v >= 0 && v < image_sizes[j].height) 
                            {
                                // 使用最近邻采样（简单高效）
                                cv::Vec3b color = images[j].at<cv::Vec3b>(v, u);
                                
                                // 创建彩色点
                                PointXYZRGBRT colored_point = point;
                                colored_point.rgba_struct.r = color[2];
                                colored_point.rgba_struct.g = color[1];
                                colored_point.rgba_struct.b = color[0];
                                colored_point.rgba_struct.a = 255;
                                local_cloud->push_back(colored_point);
                                point_colored = true;
                            }
                        }
                    }
                }
                
                // 如果点没有被任何相机看到，添加默认颜色（灰色）
                if (!point_colored) {
                    PointXYZRGBRT uncolored_point = point;
                    uncolored_point.rgba_struct.r = 128;
                    uncolored_point.rgba_struct.g = 128;
                    uncolored_point.rgba_struct.b = 128;
                    uncolored_point.rgba_struct.a = 255;
                    local_cloud->push_back(uncolored_point);
                }
            }
            
            // 合并线程本地点云
            #pragma omp critical(merge_clouds)
            {
                *colored_cloud += *local_cloud;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Colored point cloud created with %ld points (from %ld input points)", 
                colored_cloud->size(), cloud->size());
        return colored_cloud;
    }

    
    void publishColoredCloud(const pcl::PointCloud<PointXYZRGBRT>::Ptr& cloud)
    {
        PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = pointcloud_configs_[0].frame_id; // 使用第一个点云的坐标系
        
        colored_cloud_pub_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published colored point cloud with %ld points", 
                    cloud->size());
    }
    
    // 配置参数
    GeneralConfig general_config_;
    std::vector<PointCloudConfig> pointcloud_configs_;
    std::vector<ImageConfig> image_configs_;
    
    // 订阅者
    std::vector<std::shared_ptr<message_filters::Subscriber<PointCloud2>>> cloud_subs_;
    std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> image_subs_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // 服务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bag_control_service_;
    
    // 同步器
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        PointCloud2, PointCloud2, Image, Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // 发布者
    rclcpp::Publisher<PointCloud2>::SharedPtr colored_cloud_pub_;
    
    // TF相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CloudImageSyncNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}