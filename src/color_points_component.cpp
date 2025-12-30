#include "color_points_component.hpp"
#include "params_parser.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

POINT_CLOUD_REGISTER_POINT_STRUCT(color_pointscloud::PointXYZIRCAEDT, 
      (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
      (std::uint8_t, return_type, return_type)
      (float, azimuth, azimuth)(float, elevation, elevation)(float, distance, distance)
      (std::uint32_t, time_stamp, time_stamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(color_pointscloud::PointXYZRGBT, 
      (float, x, x)(float, y, y)(float, z, z)(uint32_t, rgba, rgba)
      (std::uint32_t, time_stamp, time_stamp)
)

namespace color_pointscloud
{

ColorPointsComponent::ColorPointsComponent(const rclcpp::NodeOptions & options)
: Node("color_points_component", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
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
        throw std::runtime_error("Failed to load config file");
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
  
    build_map(0);
    build_map(1);
    
    
    RCLCPP_INFO(this->get_logger(), "ColorPointsComponent initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", general_config_.output_topic.c_str());

    bag_manager_ = new BagManager(general_config_.bag_file_path, 10000);
}

ColorPointsComponent::~ColorPointsComponent()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ColorPointsComponent");
    delete bag_manager_;
}

bool ColorPointsComponent::loadConfig(const std::string& config_file)
{
    static TFQueryTool tf_query_tool;

    static  CameraParamsParser camera_params_parser;
    if (!camera_params_parser.loadFromFile("/home/pix/pix/parameter/sensor_kit/robobus_sensor_kit_description/intrinsic_parameters/camera1_params.yaml")) {
        std::cerr << "Failed to load camera YAML file!" << std::endl;
        return 1;
    }
    else{
      // 访问基础参数
        std::cout << "相机名称: " << camera_params_parser.getCameraName() << std::endl;
        std::cout << "图像尺寸: " << camera_params_parser.getImageWidth() << "x" 
                  << camera_params_parser.getImageHeight() << std::endl;
        
        // 访问相机内参矩阵
        auto camera_matrix = camera_params_parser.getCameraMatrix();
        std::cout << "相机内参矩阵 (3x3):" << std::endl;
        for (const auto& row : camera_matrix) {
            for (double val : row) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
        
        // 访问畸变系数
        auto dist_coeffs = camera_params_parser.getDistortionCoefficients();
        std::cout << "畸变系数: ";
        for (double val : dist_coeffs) {
            std::cout << val << " ";
        }
    }


    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        // 加载通用参数
        auto general = config["general"];
        general_config_.image_queue_size = general["image_queue_size"].as<int>();
        general_config_.max_time_diff = general["max_time_diff"].as<double>();
        general_config_.use_tf = general["use_tf"].as<bool>();
        general_config_.output_topic = general["output_topic"].as<std::string>();
        general_config_.bag_file_path = general["bag_file_path"].as<std::string>();
        general_config_.params_file_path = general["params_file_path"].as<std::string>();

        if (!tf_query_tool.loadYAMLFile(general_config_.params_file_path + "/extrinsic_parameters/sensor_kit_calibration.yaml")) {
            std::cerr << "Failed to load YAML file!" << std::endl;
            return false;
        }
        if (!tf_query_tool.loadYAMLFile(general_config_.params_file_path + "/extrinsic_parameters/sensors_calibration.yaml")) {
                std::cerr << "Failed to load YAML file!" << std::endl;
                return false;
            }
        
        // 加载点云配置
        pointcloud_configs_.clear();
        auto pc_topics = config["pointcloud_topics"];
        for (const auto& pc_config : pc_topics) {
            PointCloudConfig config;
            config.topic = pc_config["topic"].as<std::string>();
            config.frame_id = pc_config["frame_id"].as<std::string>();

            // 查询变换
            auto result = tf_query_tool.queryTransform("base_link", config.frame_id);
            result.print(true);
            
            config.extrinsic.rotation = result.rotation_matrix;
            config.extrinsic.translation = result.translation;
            
            // 构建变换矩阵
            config.extrinsic.transformation.setIdentity();
            config.extrinsic.transformation.block<3,3>(0,0) = config.extrinsic.rotation ;
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
            config.frame_id = img_config["frame_id"].as<std::string>();

            auto result = tf_query_tool.queryTransform(config.frame_id, "base_link");
            result.print(true);

            std::string params_file_path = general_config_.params_file_path + "/intrinsic_parameters/" + config.name + ".yaml";

            if(!camera_params_parser.loadFromFile(params_file_path)){
                std::cerr << "Failed to load camera YAML file!" << std::endl;
                return false;
            }
            
            // 解析内参
            auto intr = img_config["intrinsic_params"];
            auto mat = camera_params_parser.getCameraMatrix();
            config.intrinsic.fx = mat[0][0];
            config.intrinsic.fy = mat[1][1];
            config.intrinsic.cx = mat[0][2];
            config.intrinsic.cy = mat[1][2];
            config.intrinsic.k = camera_params_parser.getDistortionCoefficients();
            
            // 构建内参矩阵
            config.intrinsic.camera_matrix.setZero();
            config.intrinsic.camera_matrix(0,0) = config.intrinsic.fx;
            config.intrinsic.camera_matrix(1,1) = config.intrinsic.fy;
            config.intrinsic.camera_matrix(0,2) = config.intrinsic.cx;
            config.intrinsic.camera_matrix(1,2) = config.intrinsic.cy;
            config.intrinsic.camera_matrix(2,2) = 1.0;

            std::cout<<"Camera Matrix:"<<std::endl;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    std::cout << config.intrinsic.camera_matrix(i,j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "Distortion Coefficients: ";
            for (double val : config.intrinsic.k) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
            
            // 解析外参
            config.extrinsic.rotation = result.rotation_matrix;
            config.extrinsic.translation = result.translation;
            
            // 构建变换矩阵
            config.extrinsic.transformation.setIdentity();
            config.extrinsic.transformation.block<3,3>(0,0) = config.extrinsic.rotation;
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

void ColorPointsComponent::printConfigSummary()
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
    }
}

void ColorPointsComponent::initSubscribers()
{

    // 创建QoS配置
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1000));
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

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/sensing/gnss/fix", 10000,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        {
            gnss_count++;
        // std::cout << "gnss time: " << msg->header.stamp.sec << "-" << msg->header.stamp.nanosec << std::endl;
            bag_manager_->addMessage<sensor_msgs::msg::NavSatFix>(*msg, "/sensing/gnss/fix", msg->header.stamp);
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/sensing/imu/imu_data", 10000,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
        // std::cout << "imu time: " << msg->header.stamp.sec << "-" << msg->header.stamp.nanosec << std::endl;
            imu_count++;
            bag_manager_->addMessage<sensor_msgs::msg::Imu>(*msg, "/sensing/imu/imu_data", msg->header.stamp);
        });
}

void ColorPointsComponent::initSynchronizer()
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
            std::bind(&ColorPointsComponent::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3, std::placeholders::_4));
        
        RCLCPP_INFO(this->get_logger(), "Synchronizer initialized with queue size %d", 
                   general_config_.image_queue_size);
}

void ColorPointsComponent::initServices()
{
    // 创建bag记录控制服务
    bag_control_service_ = this->create_service<std_srvs::srv::SetBool>(
        "bag_control",
        std::bind(&ColorPointsComponent::handleBagControl, this,
                    std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Services initialized: bag_control");
}

void ColorPointsComponent::syncCallback(const PointCloud2::ConstSharedPtr& cloud1_msg, 
                  const PointCloud2::ConstSharedPtr& cloud2_msg, 
                  const Image::ConstSharedPtr& image1_msg, 
                  const Image::ConstSharedPtr& image2_msg)
{
    auto rosmsg_start = std::chrono::high_resolution_clock::now();
    points_count++;

    // 1. 优化时间戳处理 - 修正时间计算方式
    const auto& header1 = cloud1_msg->header;
    const auto& header2 = cloud2_msg->header;
    
    auto msg_header = header1;
    msg_header.frame_id = "base_link";
    
    uint32_t cloud1_diff_ns = 0;
    uint32_t cloud2_diff_ns = 0;
    
    // 正确的时间戳比较和计算
    const auto time1 = rclcpp::Time(header1.stamp);
    const auto time2 = rclcpp::Time(header2.stamp);
    
    if (time1 < time2) {
        msg_header.stamp = header1.stamp;
        cloud2_diff_ns = static_cast<uint32_t>((time2 - time1).nanoseconds());
    } else {
        msg_header.stamp = header2.stamp;
        cloud1_diff_ns = static_cast<uint32_t>((time1 - time2).nanoseconds());
    }

     #if 1
        const size_t size_front = cloud1_msg->width * cloud1_msg->height;
        const size_t size_rear = cloud2_msg->width * cloud2_msg->height;
        const size_t total_size = size_front + size_rear;
        
        auto cloud_conc = std::make_shared<pcl::PointCloud<PointXYZRGBT>>();
        cloud_conc->resize(total_size);
        
        // 获取外参
        const PointCloudConfig& config_front = pointcloud_configs_[0];
        const PointCloudConfig& config_rear = pointcloud_configs_[1];
        
        const Eigen::Matrix3f& Rf = config_front.extrinsic.rotation;
        const Eigen::Vector3f& Tf = config_front.extrinsic.translation;
        const Eigen::Matrix3f& Rr = config_rear.extrinsic.rotation;
        const Eigen::Vector3f& Tr = config_rear.extrinsic.translation;
        
        // 预计算矩阵元素
        const float Rf00 = Rf(0,0), Rf01 = Rf(0,1), Rf02 = Rf(0,2);
        const float Rf10 = Rf(1,0), Rf11 = Rf(1,1), Rf12 = Rf(1,2);
        const float Rf20 = Rf(2,0), Rf21 = Rf(2,1), Rf22 = Rf(2,2);
        
        const float Rr00 = Rr(0,0), Rr01 = Rr(0,1), Rr02 = Rr(0,2);
        const float Rr10 = Rr(1,0), Rr11 = Rr(1,1), Rr12 = Rr(1,2);
        const float Rr20 = Rr(2,0), Rr21 = Rr(2,1), Rr22 = Rr(2,2);
        
        const float Tfx = Tf.x(), Tfy = Tf.y(), Tfz = Tf.z();
        const float Trx = Tr.x(), Try = Tr.y(), Trz = Tr.z();
        
        // 获取点云字段信息
        auto getFieldOffset = [](const sensor_msgs::msg::PointCloud2& cloud, 
                                const std::string& field_name) -> int {
            for (const auto& field : cloud.fields) {
                if (field.name == field_name) return field.offset;
            }
            return -1;
        };
        
        int x_off1 = getFieldOffset(*cloud1_msg, "x");
        int y_off1 = getFieldOffset(*cloud1_msg, "y");
        int z_off1 = getFieldOffset(*cloud1_msg, "z");
        int t_off1 = getFieldOffset(*cloud1_msg, "time_stamp");
        
        int x_off2 = getFieldOffset(*cloud2_msg, "x");
        int y_off2 = getFieldOffset(*cloud2_msg, "y");
        int z_off2 = getFieldOffset(*cloud2_msg, "z");
        int t_off2 = getFieldOffset(*cloud2_msg, "time_stamp");
        
        // 验证字段
        if (x_off1 == -1 || y_off1 == -1 || z_off1 == -1 || t_off1 == -1 ||
            x_off2 == -1 || y_off2 == -1 || z_off2 == -1 || t_off2 == -1) {
            RCLCPP_ERROR(this->get_logger(), "Missing required point cloud fields");
            return;
        }
        
        const uint8_t* data1 = cloud1_msg->data.data();
        const uint8_t* data2 = cloud2_msg->data.data();
        const uint32_t point_step1 = cloud1_msg->point_step;
        const uint32_t point_step2 = cloud2_msg->point_step;
        
        // 并行处理前部点云
        #pragma omp parallel for
        for (size_t i = 0; i < size_front; ++i) {
            const uint8_t* point_ptr = data1 + i * point_step1;
            
            float x, y, z;
            uint32_t timestamp;
            
            memcpy(&x, point_ptr + x_off1, sizeof(float));
            memcpy(&y, point_ptr + y_off1, sizeof(float));
            memcpy(&z, point_ptr + z_off1, sizeof(float));
            memcpy(&timestamp, point_ptr + t_off1, sizeof(uint32_t));
            
            auto& dst = cloud_conc->points[i];
            dst.x = Rf00*x + Rf01*y + Rf02*z + Tfx;
            dst.y = Rf10*x + Rf11*y + Rf12*z + Tfy;
            dst.z = Rf20*x + Rf21*y + Rf22*z + Tfz;
            dst.time_stamp = timestamp + cloud1_diff_ns;
        }
        
        // 并行处理后部点云
        #pragma omp parallel for
        for (size_t i = 0; i < size_rear; ++i) {
            const uint8_t* point_ptr = data2 + i * point_step2;
            
            float x, y, z;
            uint32_t timestamp;
            
            memcpy(&x, point_ptr + x_off2, sizeof(float));
            memcpy(&y, point_ptr + y_off2, sizeof(float));
            memcpy(&z, point_ptr + z_off2, sizeof(float));
            memcpy(&timestamp, point_ptr + t_off2, sizeof(uint32_t));
            
            auto& dst = cloud_conc->points[i + size_front];
            dst.x = Rr00*x + Rr01*y + Rr02*z + Trx;
            dst.y = Rr10*x + Rr11*y + Rr12*z + Try;
            dst.z = Rr20*x + Rr21*y + Rr22*z + Trz;
            dst.time_stamp = timestamp + cloud2_diff_ns;
        }
    #endif

    auto rosmsg_2 = std::chrono::high_resolution_clock::now();

    
    // 优化4: 并行处理图像
    std::vector<cv::Mat> images(2);
    #if 1
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                images[0] = processJPEGImage(image1_msg, 0);
            }
            
            #pragma omp section
            {
                images[1] = processJPEGImage(image2_msg, 1);
            }
        }
    #else
        images[0] = processRGBAImage(image1_msg);
        images[1] = processRGBAImage(image2_msg);
    #endif
    
    
    auto rosmsg_3 = std::chrono::high_resolution_clock::now();

    // 优化5: 使用引用避免拷贝
    const auto& colored_cloud = colorPointCloud(cloud_conc, images);
    
    // 优化6: 直接发布，避免中间转换
    sensor_msgs::msg::PointCloud2 colored_cloud_msg;
    pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
    colored_cloud_msg.header = msg_header;

    colored_cloud_pub_->publish(colored_cloud_msg);
    
    // 写入Bag文件
    bag_manager_->addMessage<sensor_msgs::msg::PointCloud2>(
        colored_cloud_msg,
        "/sensing/lidar/points_rgb",
        msg_header.stamp
    );
    auto color_end = std::chrono::high_resolution_clock::now();
    std::cout << "time_trans: " << std::chrono::duration_cast<std::chrono::milliseconds>(rosmsg_2 - rosmsg_start).count() << "ms " << 
    "time_undistort: " << std::chrono::duration_cast<std::chrono::milliseconds>(rosmsg_3 - rosmsg_2).count() << "ms " << 
    "time_color: " << std::chrono::duration_cast<std::chrono::milliseconds>(color_end - rosmsg_3).count() << "ms " << 
    " points count: " << points_count <<" imu count: " << imu_count << " gnss count: " << gnss_count << std::endl;
}

void ColorPointsComponent::handleBagControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
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


// 辅助函数：计算点的水平角度（相对于X轴）
inline float calculateAzimuth(const PointXYZRGBT& point) {
    return std::atan2(point.y, point.x) * 180.0f / M_PI;  // 转换为度
}

// 辅助函数：判断点是否在需要着色的角度范围内
inline bool isPointInColorRange(const PointXYZRGBT& point, 
                                float min_angle = -50.0f, 
                                float max_angle = 50.0f) {
    // 计算点的方位角（-180° 到 180°）
    float azimuth = std::atan2(point.y, point.x) * 180.0f / M_PI;
    
    // 处理角度标准化到 [-180, 180] 范围
    if (azimuth > 180.0f) azimuth -= 360.0f;
    if (azimuth < -180.0f) azimuth += 360.0f;
    
    // 检查是否在第一个范围：±50度
    if (azimuth >= min_angle && azimuth <= max_angle) {
        return true;
    }
    
    // 检查是否在第二个范围：130到-130度
    // 注意：130到-130度实际上是一个跨越±180度的范围
    if (max_angle >= 130.0f && min_angle <= -130.0f) {
        // 130到-130度实际上包括两部分：130到180度和-180到-130度
        return (azimuth >= 130.0f || azimuth <= -130.0f);
    } else {
        // 正常范围检查
        return (azimuth >= 130.0f && azimuth <= -130.0f);
    }
}

pcl::PointCloud<PointXYZRGBT>::Ptr ColorPointsComponent::colorPointCloud_V1(
    const pcl::PointCloud<PointXYZRGBT>::Ptr& cloud, 
    const std::vector<cv::Mat>& images)
{
    // 创建新的点云并拷贝原始数据
    auto colored_cloud = pcl::PointCloud<PointXYZRGBT>::Ptr(new pcl::PointCloud<PointXYZRGBT>());
    
    // 设置与输入点云相同的大小，并复制所有点
    *colored_cloud = *cloud;  
    
    // 检查图像有效性
    if (images.size() != 2 || image_configs_.size() != 2) {
        RCLCPP_ERROR(this->get_logger(), "V2: 输入图像数量必须为2, 且对应相机配置数量也必须为2");
        return colored_cloud;
    }
    
    bool img0_valid = !images[0].empty();
    bool img1_valid = !images[1].empty();
    cv::Size img0_size = images[0].size();
    cv::Size img1_size = images[1].size();
    
    if (!img0_valid || !img1_valid) {
        RCLCPP_WARN(this->get_logger(), "图像为空，跳过着色");
        return colored_cloud;
    }
    
    // 获取相机的内参和外参
    const IntrinsicParams* intrinsic0 = &image_configs_[0].intrinsic;
    const IntrinsicParams* intrinsic1 = &image_configs_[1].intrinsic;
    const ExtrinsicParams* extrinsic0 = &image_configs_[0].extrinsic;
    const ExtrinsicParams* extrinsic1 = &image_configs_[1].extrinsic;
    
    // 预计算变换矩阵的旋转和平移部分
    Eigen::Matrix3f R0 = extrinsic0->transformation.block<3,3>(0,0);
    Eigen::Vector3f t0 = extrinsic0->transformation.block<3,1>(0,3);
    Eigen::Matrix3f R1 = extrinsic1->transformation.block<3,3>(0,0);
    Eigen::Vector3f t1 = extrinsic1->transformation.block<3,1>(0,3);
    
    // ==================== 优化1: 预计算投影矩阵 ====================
    // 内参矩阵 K
    Eigen::Matrix3f K0, K1;
    K0 << intrinsic0->fx, 0,          intrinsic0->cx,
          0,          intrinsic0->fy, intrinsic0->cy,
          0,          0,          1;
    
    K1 << intrinsic1->fx, 0,          intrinsic1->cx,
          0,          intrinsic1->fy, intrinsic1->cy,
          0,          0,          1;
    
    // 完整的投影矩阵 P = K * [R|t]
    Eigen::Matrix<float, 3, 4> P0, P1;
    P0.block<3,3>(0,0) = K0 * R0;
    P0.block<3,1>(0,3) = K0 * t0;
    P1.block<3,3>(0,0) = K1 * R1;
    P1.block<3,1>(0,3) = K1 * t1;
    
    // 图像尺寸边界（用于快速边界检查）
    int img0_width = img0_size.width;
    int img0_height = img0_size.height;
    int img1_width = img1_size.width;
    int img1_height = img1_size.height;
    
    // ==================== 优化2: 内存访问优化 ====================
    // 获取原始数据指针
    const PointXYZRGBT* cloud_data = cloud->points.data();
    PointXYZRGBT* colored_data = colored_cloud->points.data();
    const size_t point_count = cloud->size();
    
    // 获取图像数据指针
    const cv::Mat& img0 = images[0];
    const cv::Mat& img1 = images[1];
    
    // 检查图像类型是否为BGR
    if (img0.type() != CV_8UC3 || img1.type() != CV_8UC3) {
        RCLCPP_ERROR(this->get_logger(), "图像必须是CV_8UC3 (BGR)格式");
        return colored_cloud;
    }
    
    const uchar* img0_data = img0.data;
    const uchar* img1_data = img1.data;
    const int img0_step = img0.step;  // 每行的字节数
    const int img1_step = img1.step;
    
    // ==================== 新增: 角度范围过滤 ====================
    // 定义需要着色的角度范围
    const float angle_range1_min = -42.5f;  // ±50度
    const float angle_range1_max = 42.5f;
    const float angle_range2_min = 137.5f;  // 130到-130度
    const float angle_range2_max = -137.5f;
    
    // 预计算角度的sin/cos值，避免重复计算
    const float cos_50 = std::cos(40.0f * M_PI / 180.0f);
    const float sin_130 = std::sin(135.0f * M_PI / 180.0f);
    const float cos_130 = std::cos(135.0f * M_PI / 180.0f);
    
    // 统计
    size_t colored_count = 0;
    size_t skipped_by_angle = 0;
    
    // 使用动态调度，块大小设为256以提高缓存利用率
    #pragma omp parallel for schedule(dynamic, 256) reduction(+:colored_count, skipped_by_angle)
    for (size_t i = 0; i < point_count; ++i) 
    {
        const auto& point = cloud_data[i];
        auto& colored_point = colored_data[i];
        
        // ==================== 新增: 角度过滤判断 ====================
        // 方法1: 使用快速角度检查（避免atan2计算）
        float x = point.x;
        float y = point.y;
        float norm_sq = x*x + y*y;
        
        if (norm_sq < 1e-6f) {
            // 点接近原点，直接设置默认颜色
            colored_point.rgba_struct.r = 255;
            colored_point.rgba_struct.g = 0;
            colored_point.rgba_struct.b = 0;
            colored_point.rgba_struct.a = 255;
            skipped_by_angle++;
            continue;
        }
        
        // 计算点的单位向量
        float inv_norm = 1.0f / std::sqrt(norm_sq);
        float nx = x * inv_norm;
        float ny = y * inv_norm;
        
        // 检查是否在±50度范围内（X轴正方向）
        // 相当于检查点与X轴正方向的夹角是否小于50度
        if (nx >= cos_50) {  // cos(θ) = nx，当θ<50°时，cosθ>cos50°
            // 在±50度范围内，需要处理
        } 
        // 检查是否在130到-130度范围内（大致是X轴负方向）
        else if (nx <= cos_130 && ny >= -sin_130 && ny <= sin_130) {
            // 在130到-130度范围内，需要处理
        }
        else {
            // 不在需要着色的角度范围内，设置默认颜色
            colored_point.rgba_struct.r = 0;
            colored_point.rgba_struct.g = 255;
            colored_point.rgba_struct.b = 0;
            colored_point.rgba_struct.a = 255;
            skipped_by_angle++;
            continue;
        }
        
        // ==================== 原着色逻辑 ====================
        // 根据x坐标选择图像索引
        int img_idx = (point.x >= 0.0f) ? 0 : 1;
        
        // 选择对应的投影矩阵和图像参数
        const Eigen::Matrix<float, 3, 4>& P = (img_idx == 0) ? P0 : P1;
        const uchar* img_data = (img_idx == 0) ? img0_data : img1_data;
        const int img_step = (img_idx == 0) ? img0_step : img1_step;
        const int img_width = (img_idx == 0) ? img0_width : img1_width;
        const int img_height = (img_idx == 0) ? img0_height : img1_height;
        
        // 使用齐次坐标投影：pixel = P * [X; Y; Z; 1]
        const float z = point.z;
        
        // 计算投影坐标
        const float u_homo = P(0,0)*x + P(0,1)*y + P(0,2)*z + P(0,3);
        const float v_homo = P(1,0)*x + P(1,1)*y + P(1,2)*z + P(1,3);
        const float w_homo = P(2,0)*x + P(2,1)*y + P(2,2)*z + P(2,3);
        
        // 检查点是否在相机前方
        if (w_homo > 0.1f) 
        {
            // 透视除法
            const float inv_w = 1.0f / w_homo;
            const float u_f = u_homo * inv_w;
            const float v_f = v_homo * inv_w;
            
            // 转换为整数像素坐标
            const int u = static_cast<int>(u_f);
            const int v = static_cast<int>(v_f);
            
            // 边界检查
            if (u >= 0 && u < img_width && v >= 0 && v < img_height) 
            {
                // 直接内存访问获取像素值
                const uchar* pixel_ptr = img_data + v * img_step + u * 3;
                
                // BGR格式：注意OpenCV的BGR顺序
                colored_point.rgba_struct.b = pixel_ptr[0];  // Blue
                colored_point.rgba_struct.g = pixel_ptr[1];  // Green
                colored_point.rgba_struct.r = pixel_ptr[2];  // Red
                colored_point.rgba_struct.a = 255;
                
                colored_count++;
                continue;  // 跳过默认颜色设置
            }
        }
        
        // 如果点没有被成功着色，设置默认颜色（红色）
        colored_point.rgba_struct.r = 255;
        colored_point.rgba_struct.g = 0;
        colored_point.rgba_struct.b = 0;
        colored_point.rgba_struct.a = 255;
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "角度过滤跳过: %zu, 成功着色: %zu/%zu (%.1f%%)", 
                 skipped_by_angle, colored_count, point_count, 
                 100.0 * colored_count / point_count);
    
    return colored_cloud;
}

pcl::PointCloud<PointXYZRGBT>::Ptr ColorPointsComponent::colorPointCloud(
    const pcl::PointCloud<PointXYZRGBT>::Ptr& cloud, 
    const std::vector<cv::Mat>& images)
{
    // 创建新的点云并拷贝原始数据
    auto colored_cloud = pcl::PointCloud<PointXYZRGBT>::Ptr(new pcl::PointCloud<PointXYZRGBT>());
    
    // 设置与输入点云相同的大小，并复制所有点
    *colored_cloud = *cloud;  
    
    // 检查图像有效性
    if (images.size() != 2 || image_configs_.size() != 2) {
        RCLCPP_ERROR(this->get_logger(), "V2: 输入图像数量必须为2, 且对应相机配置数量也必须为2");
        return colored_cloud;
    }
    
    bool img0_valid = !images[0].empty();
    bool img1_valid = !images[1].empty();
    cv::Size img0_size = images[0].size();
    cv::Size img1_size = images[1].size();
    
    if (!img0_valid || !img1_valid) {
        RCLCPP_WARN(this->get_logger(), "图像为空，跳过着色");
        return colored_cloud;
    }
    
    // 获取相机的内参和外参
    const IntrinsicParams* intrinsic0 = &image_configs_[0].intrinsic;
    const IntrinsicParams* intrinsic1 = &image_configs_[1].intrinsic;
    const ExtrinsicParams* extrinsic0 = &image_configs_[0].extrinsic;
    const ExtrinsicParams* extrinsic1 = &image_configs_[1].extrinsic;
    
    // 预计算变换矩阵的旋转和平移部分
    Eigen::Matrix3f R0 = extrinsic0->transformation.block<3,3>(0,0);
    Eigen::Vector3f t0 = extrinsic0->transformation.block<3,1>(0,3);
    Eigen::Matrix3f R1 = extrinsic1->transformation.block<3,3>(0,0);
    Eigen::Vector3f t1 = extrinsic1->transformation.block<3,1>(0,3);
    
    // ==================== 优化1: 预计算投影矩阵 ====================
    // 内参矩阵 K
    Eigen::Matrix3f K0, K1;
    K0 << intrinsic0->fx, 0,          intrinsic0->cx,
          0,          intrinsic0->fy, intrinsic0->cy,
          0,          0,          1;
    
    K1 << intrinsic1->fx, 0,          intrinsic1->cx,
          0,          intrinsic1->fy, intrinsic1->cy,
          0,          0,          1;
    
    // 完整的投影矩阵 P = K * [R|t]
    Eigen::Matrix<float, 3, 4> P0, P1;
    P0.block<3,3>(0,0) = K0 * R0;
    P0.block<3,1>(0,3) = K0 * t0;
    P1.block<3,3>(0,0) = K1 * R1;
    P1.block<3,1>(0,3) = K1 * t1;
    
    // 图像尺寸边界（用于快速边界检查）
    int img0_width = img0_size.width;
    int img0_height = img0_size.height;
    int img1_width = img1_size.width;
    int img1_height = img1_size.height;
    
    // ==================== 优化2: 内存访问优化 ====================
    // 获取原始数据指针
    const PointXYZRGBT* cloud_data = cloud->points.data();
    PointXYZRGBT* colored_data = colored_cloud->points.data();
    const size_t point_count = cloud->size();
    
    // 获取图像数据指针
    const cv::Mat& img0 = images[0];
    const cv::Mat& img1 = images[1];
    
    // 检查图像类型是否为BGR
    if (img0.type() != CV_8UC3 || img1.type() != CV_8UC3) {
        RCLCPP_ERROR(this->get_logger(), "图像必须是CV_8UC3 (BGR)格式");
        return colored_cloud;
    }
    
    const uchar* img0_data = img0.data;
    const uchar* img1_data = img1.data;
    const int img0_step = img0.step;  // 每行的字节数
    const int img1_step = img1.step;
    
    // 统计成功着色的点数
    size_t colored_count = 0;
    
    // 使用动态调度，块大小设为256以提高缓存利用率
    #pragma omp parallel for schedule(dynamic, 256) reduction(+:colored_count)
    for (size_t i = 0; i < point_count; ++i) 
    {
        const auto& point = cloud_data[i];
        auto& colored_point = colored_data[i];
        
        // 根据x坐标选择图像索引
        int img_idx = (point.x >= 0.0f) ? 0 : 1;
        
        // 选择对应的投影矩阵和图像参数
        const Eigen::Matrix<float, 3, 4>& P = (img_idx == 0) ? P0 : P1;
        const uchar* img_data = (img_idx == 0) ? img0_data : img1_data;
        const int img_step = (img_idx == 0) ? img0_step : img1_step;
        const int img_width = (img_idx == 0) ? img0_width : img1_width;
        const int img_height = (img_idx == 0) ? img0_height : img1_height;
        
        // 使用齐次坐标投影：pixel = P * [X; Y; Z; 1]
        // 展开矩阵乘法以提高效率
        const float x = point.x;
        const float y = point.y;
        const float z = point.z;
        
        // 计算投影坐标
        const float u_homo = P(0,0)*x + P(0,1)*y + P(0,2)*z + P(0,3);
        const float v_homo = P(1,0)*x + P(1,1)*y + P(1,2)*z + P(1,3);
        const float w_homo = P(2,0)*x + P(2,1)*y + P(2,2)*z + P(2,3);
        
        // 检查点是否在相机前方
        if (w_homo > 0.1f) 
        {
            // 透视除法
            const float inv_w = 1.0f / w_homo;
            const float u_f = u_homo * inv_w;
            const float v_f = v_homo * inv_w;
            
            // 转换为整数像素坐标
            const int u = static_cast<int>(u_f);
            const int v = static_cast<int>(v_f);
            
            // 边界检查
            if (u >= 0 && u < img_width && v >= 0 && v < img_height) 
            {
                // 直接内存访问获取像素值
                const uchar* pixel_ptr = img_data + v * img_step + u * 3;
                
                // BGR格式：注意OpenCV的BGR顺序
                colored_point.rgba_struct.b = pixel_ptr[0];  // Blue
                colored_point.rgba_struct.g = pixel_ptr[1];  // Green
                colored_point.rgba_struct.r = pixel_ptr[2];  // Red
                colored_point.rgba_struct.a = 255;
                
                colored_count++;
                continue;  // 跳过默认颜色设置
            }
        }
        
        // 如果点没有被成功着色，设置默认颜色（红色）
        colored_point.rgba_struct.r = 255;
        colored_point.rgba_struct.g = 0;
        colored_point.rgba_struct.b = 0;
        colored_point.rgba_struct.a = 255;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "成功着色 %zu/%zu 个点 (%.1f%%)", 
                 colored_count, point_count, 
                 100.0 * colored_count / point_count);
    
    return colored_cloud;
}
    


void ColorPointsComponent::build_map(const int index)
{
    // 去畸变
    IntrinsicParams intr_params = image_configs_[index].intrinsic;
    
    
    // 创建OpenCV相机矩阵
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        intr_params.fx, 0, intr_params.cx,
        0, intr_params.fy, intr_params.cy,
        0, 0, 1);
    
    // 将畸变系数转换为cv::Mat
    cv::Mat dist_coeffs;
    if (intr_params.k.size() == 8) 
    {
        
        if(intr_params.k[5] + intr_params.k[6] == 0.0 + intr_params.k[7] < 1e-5)
        {
            dist_coeffs = cv::Mat_<double>(1, 5);
            for (size_t i = 0; i < 5 && i < intr_params.k.size(); ++i) 
            {
                dist_coeffs.at<double>(0, i) = intr_params.k[i];
            }
        }
        else
        {
            // 8参数畸变模型 [k1, k2, p1, p2, k3, k4, k5, k6]
            dist_coeffs = cv::Mat_<double>(1, 8);
            for (size_t i = 0; i < 8; ++i) 
            {
                dist_coeffs.at<double>(0, i) = intr_params.k[i];
            }
        }
        
    } 
    else if (intr_params.k.size() == 5) 
    {
        // 至少需要5个参数 [k1, k2, p1, p2, k3]
        dist_coeffs = cv::Mat_<double>(1, 5);
        for (size_t i = 0; i < 5 && i < intr_params.k.size(); ++i) 
        {
            dist_coeffs.at<double>(0, i) = intr_params.k[i];
        }
    }
    
    // 检查是否需要初始化映射表
    if (undistort_maps_[index].first.empty() || undistort_maps_[index].second.empty()) {
        
        // 初始化映射表（使用原始内参保持内参不变）
        cv::Mat new_camera_matrix = camera_matrix.clone();
        cv::Mat map1, map2;
        
        cv::initUndistortRectifyMap(
            camera_matrix,        // 原始相机内参
            dist_coeffs,          // 畸变系数
            cv::Mat(),            // 无旋转矩阵
            new_camera_matrix,    // 使用相同的内参矩阵（保持内参不变）
            cv::Size(1920, 1080),         // 输出尺寸与输入相同
            CV_32FC1,             // 映射表数据类型
            map1, map2           // 输出的映射表
        );
        
        // 缓存映射表和图像尺寸
        undistort_maps_[index] = std::make_pair(map1, map2);
        image_size_cache_[index] = cv::Size(1920, 1080);

    }
        
}

cv::Mat ColorPointsComponent::processJPEGImage(const Image::ConstSharedPtr& image_msg, size_t index)
{
    cv::Mat image;
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = cv_ptr->image;
        cv::Mat undistorted = cv::Mat::zeros(image_size_cache_[index], image.type());
        
        // 应用去畸变映射
        cv::remap(image, undistorted, 
                undistort_maps_[index].first, 
                undistort_maps_[index].second,
                cv::INTER_LINEAR, 
                cv::BORDER_CONSTANT);
        
        image = undistorted;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error processing image %zu: %s", index, e.what());
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error processing image %zu: %s", index, e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing image %zu: %s", index, e.what());
    }
    return image;
}

// cv::Mat ColorPointsComponent::processRGBAImage(const Image::ConstSharedPtr& image_msg)
// {
//     cv_bridge::CvImageConstPtr cv_ptr;
//      // 1. 如果本身就是 bgr8，直接返回（零拷贝）
//     if (image_msg->encoding == "bgr8" || image_msg->encoding == "rgb8") 
//     {
//         cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
//         return cv_ptr->image.clone();
//     }
//     else 
//     {
//         RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", image_msg->encoding.c_str());
//     }
//     return cv::Mat();
// }


} // namespace color_pointscloud

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(color_pointscloud::ColorPointsComponent)