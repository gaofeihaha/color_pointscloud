#ifndef PARAMS_PARSER_HPP_
#define PARAMS_PARSER_HPP_

#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <algorithm>

class TFQueryTool {
public:
    struct TransformResult {
        Eigen::Vector3f translation;      // 平移: x, y, z (float)
        Eigen::Quaternionf rotation;       // 四元数旋转 (float)
        Eigen::Matrix3f rotation_matrix;   // 旋转矩阵 (float)
        Eigen::Vector3f euler_angles;      // 欧拉角: roll, pitch, yaw (弧度, float)
        std::string parent_frame;
        std::string child_frame;
        
        TransformResult() = default;
        
        TransformResult(const std::string& parent, const std::string& child)
            : parent_frame(parent), child_frame(child) {}
        
        void print(bool verbose = false) const {
            std::cout << "\n=== Transform: " << parent_frame << " -> " << child_frame << " ===" << std::endl;
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "Translation (x, y, z):" << std::endl;
            std::cout << "  " << translation.x() << ", " 
                      << translation.y() << ", " 
                      << translation.z() << std::endl;
            
            std::cout << "Rotation matrix:" << std::endl;
            std::cout << "  " << rotation_matrix(0,0) << ", " 
                      << rotation_matrix(0,1) << ", " 
                      << rotation_matrix(0,2) << std::endl;
            std::cout << "  " << rotation_matrix(1,0) << ", " 
                      << rotation_matrix(1,1) << ", " 
                      << rotation_matrix(1,2) << std::endl;
            std::cout << "  " << rotation_matrix(2,0) << ", " 
                      << rotation_matrix(2,1) << ", " 
                      << rotation_matrix(2,2) << std::endl;
            
            if (verbose) {
                std::cout << "Euler angles (roll, pitch, yaw in radians):" << std::endl;
                std::cout << "  " << euler_angles.x() << ", " 
                          << euler_angles.y() << ", " 
                          << euler_angles.z() << std::endl;
                
                std::cout << "Euler angles (roll, pitch, yaw in degrees):" << std::endl;
                std::cout << "  " << euler_angles.x() * 180.0f / M_PI << ", " 
                          << euler_angles.y() * 180.0f / M_PI << ", " 
                          << euler_angles.z() * 180.0f / M_PI << std::endl;
            }
        }
    };

private:
    tf2::BufferCore buffer_;
    rclcpp::Time fixed_time_;
    std::set<std::string> known_frames_;
    std::vector<std::string> loaded_files_;
    
    // 将欧拉角转换为四元数 (RPY: roll, pitch, yaw) - 使用 float
    Eigen::Quaternionf eulerToQuaternion(float roll, float pitch, float yaw) {
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        return q.normalized();
    }
    
    // 将四元数转换为欧拉角 - 使用 float
    Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf& q) {
        Eigen::Matrix3f m = q.toRotationMatrix();
        float roll = std::atan2(m(2, 1), m(2, 2));
        float pitch = std::asin(-m(2, 0));
        float yaw = std::atan2(m(1, 0), m(0, 0));
        return Eigen::Vector3f(roll, pitch, yaw);
    }
    
    // 从 YAML 节点递归加载坐标变换
    void loadNodeRecursive(const YAML::Node& node, const std::string& parent_frame, 
                          const std::map<std::string, std::vector<float>>& global_offset = {}) {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            std::string child_frame = it->first.as<std::string>();
            
            if (it->second.IsMap()) {
                // 检查是否是变换节点
                if (it->second["x"] && it->second["y"] && it->second["z"] &&
                    it->second["roll"] && it->second["pitch"] && it->second["yaw"]) {
                    
                    // 读取变换参数 - 转换为 float
                    float x = static_cast<float>(it->second["x"].as<double>());
                    float y = static_cast<float>(it->second["y"].as<double>());
                    float z = static_cast<float>(it->second["z"].as<double>());
                    float roll = static_cast<float>(it->second["roll"].as<double>());
                    float pitch = static_cast<float>(it->second["pitch"].as<double>());
                    float yaw = static_cast<float>(it->second["yaw"].as<double>());
                    
                    // 应用全局偏移
                    auto offset_it = global_offset.find(child_frame);
                    if (offset_it != global_offset.end() && offset_it->second.size() >= 6) {
                        const auto& offset = offset_it->second;
                        x += offset[0];
                        y += offset[1];
                        z += offset[2];
                        roll += offset[3];
                        pitch += offset[4];
                        yaw += offset[5];
                    }
                    
                    // 创建变换 - 使用固定的时间戳
                    geometry_msgs::msg::TransformStamped transform;
                    transform.header.stamp = fixed_time_;
                    transform.header.frame_id = parent_frame;
                    transform.child_frame_id = child_frame;
                    
                    // 设置平移
                    transform.transform.translation.x = x;
                    transform.transform.translation.y = y;
                    transform.transform.translation.z = z;
                    
                    // 设置旋转
                    Eigen::Quaternionf q = eulerToQuaternion(roll, pitch, yaw);
                    transform.transform.rotation.x = q.x();
                    transform.transform.rotation.y = q.y();
                    transform.transform.rotation.z = q.z();
                    transform.transform.rotation.w = q.w();
                    
                    // 添加到 Buffer
                    buffer_.setTransform(transform, "static");
                    
                    // 添加到已知坐标系列表
                    if (!parent_frame.empty()) known_frames_.insert(parent_frame);
                    known_frames_.insert(child_frame);
                    
                    std::cout << "Loaded: " << parent_frame << " -> " << child_frame 
                              << " [x:" << x << " y:" << y << " z:" << z 
                              << " r:" << roll << " p:" << pitch << " y:" << yaw << "]" << std::endl;
                    
                    // 递归处理子节点
                    loadNodeRecursive(it->second, child_frame);
                } else {
                    // 直接递归处理
                    loadNodeRecursive(it->second, child_frame);
                }
            }
        }
    }

public:
    TFQueryTool() {
        // 使用一个固定的时间戳，避免时间戳问题
        fixed_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    
    // 加载 YAML 文件
    bool loadYAMLFile(const std::string& filepath, 
                     const std::map<std::string, std::vector<float>>& global_offset = {}) {
        try {
            std::cout << "\nLoading YAML: " << filepath << std::endl;
            YAML::Node config = YAML::LoadFile(filepath);
            loadNodeRecursive(config, "", global_offset);
            loaded_files_.push_back(filepath);
            std::cout << "Successfully loaded " << known_frames_.size() << " frames" << std::endl;
            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML error: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }
    
    // 查询坐标系之间的变换
    TransformResult queryTransform(const std::string& target_frame, 
                                  const std::string& source_frame) {
        TransformResult result(target_frame, source_frame);
        
        try {
            // 使用固定时间戳查询
            auto tf_stamped = buffer_.lookupTransform(
                target_frame, 
                source_frame, 
                tf2::timeFromSec(fixed_time_.seconds())
            );
            
            // 提取变换数据 - 转换为 float
            result.translation = Eigen::Vector3f(
                static_cast<float>(tf_stamped.transform.translation.x),
                static_cast<float>(tf_stamped.transform.translation.y),
                static_cast<float>(tf_stamped.transform.translation.z)
            );
            
            result.rotation = Eigen::Quaternionf(
                static_cast<float>(tf_stamped.transform.rotation.w),
                static_cast<float>(tf_stamped.transform.rotation.x),
                static_cast<float>(tf_stamped.transform.rotation.y),
                static_cast<float>(tf_stamped.transform.rotation.z)
            );
            
            result.euler_angles = quaternionToEuler(result.rotation);
            result.rotation_matrix = result.rotation.toRotationMatrix();
            
            return result;
            
        } catch (const tf2::TransformException& e) {
            throw std::runtime_error("Failed to query transform from '" + 
                                    source_frame + "' to '" + target_frame + 
                                    "': " + std::string(e.what()));
        }
    }
    
    // 检查是否可查询变换
    bool canTransform(const std::string& target_frame, 
                     const std::string& source_frame) {
        return buffer_.canTransform(target_frame, source_frame, 
                                   tf2::timeFromSec(fixed_time_.seconds()));
    }
    
    // 获取所有已加载的坐标系
    std::vector<std::string> getAllFrames() {
        return std::vector<std::string>(known_frames_.begin(), known_frames_.end());
    }
    
    // 查找所有子坐标系
    std::vector<std::string> getChildFrames(const std::string& parent_frame) {
        std::vector<std::string> children;
        for (const auto& frame : known_frames_) {
            if (frame != parent_frame) {
                try {
                    buffer_.lookupTransform(parent_frame, frame, 
                                          tf2::timeFromSec(fixed_time_.seconds()));
                    children.push_back(frame);
                } catch (...) {
                    // 查询失败，不是子坐标系
                }
            }
        }
        return children;
    }
    
    // 打印 TF 树结构
    void printTFTree(const std::string& root_frame = "") {
        std::cout << "\n=== TF Tree (" << known_frames_.size() << " frames) ===" << std::endl;
        
        if (!root_frame.empty() && known_frames_.find(root_frame) != known_frames_.end()) {
            std::cout << "Root: " << root_frame << std::endl;
            printSubtree(root_frame, 0);
        } else {
            // 找到可能的根节点
            std::set<std::string> possible_roots = known_frames_;
            for (const auto& frame : known_frames_) {
                for (const auto& other : known_frames_) {
                    if (frame != other && canTransform(other, frame)) {
                        possible_roots.erase(frame);
                        break;
                    }
                }
            }
            
            for (const auto& root : possible_roots) {
                printSubtree(root, 0);
            }
        }
    }
    
    // 递归打印子树
    void printSubtree(const std::string& frame, int depth) {
        std::string indent(depth * 2, ' ');
        std::cout << indent << (depth == 0 ? "├── " : "└── ") << frame;
        
        auto children = getChildFrames(frame);
        if (!children.empty()) {
            std::cout << " (" << children.size() << " children)" << std::endl;
            for (const auto& child : children) {
                printSubtree(child, depth + 1);
            }
        } else {
            std::cout << " (leaf)" << std::endl;
        }
    }
    
    // 获取已加载的文件列表
    const std::vector<std::string>& getLoadedFiles() const {
        return loaded_files_;
    }
    
    // 清除所有变换
    void clear() {
        buffer_.clear();
        known_frames_.clear();
        loaded_files_.clear();
        std::cout << "All transforms cleared." << std::endl;
    }
    
    // 批量查询变换
    std::vector<TransformResult> batchQuery(
        const std::vector<std::pair<std::string, std::string>>& queries) {
        std::vector<TransformResult> results;
        for (const auto& query : queries) {
            try {
                results.push_back(queryTransform(query.first, query.second));
            } catch (const std::exception& e) {
                std::cerr << "Query error [" << query.first << " -> " 
                          << query.second << "]: " << e.what() << std::endl;
            }
        }
        return results;
    }
    
    // 手动添加变换 - 使用 float
    void addTransform(const std::string& parent_frame,
                     const std::string& child_frame,
                     const Eigen::Vector3f& translation,
                     const Eigen::Quaternionf& rotation) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = fixed_time_;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        
        transform.transform.rotation.x = rotation.x();
        transform.transform.rotation.y = rotation.y();
        transform.transform.rotation.z = rotation.z();
        transform.transform.rotation.w = rotation.w();
        
        buffer_.setTransform(transform, "static");
        
        if (!parent_frame.empty()) known_frames_.insert(parent_frame);
        known_frames_.insert(child_frame);
        
        std::cout << "Added: " << parent_frame << " -> " << child_frame << std::endl;
    }
    
    // 从欧拉角添加变换 - 使用 float
    void addTransformFromEuler(const std::string& parent_frame,
                              const std::string& child_frame,
                              float x, float y, float z,
                              float roll, float pitch, float yaw) {
        Eigen::Quaternionf q = eulerToQuaternion(roll, pitch, yaw);
        addTransform(parent_frame, child_frame, 
                    Eigen::Vector3f(x, y, z), q);
    }
    
    // 获取变换矩阵 - 使用 float
    Eigen::Matrix4f getTransformMatrix(const std::string& target_frame,
                                      const std::string& source_frame) {
        TransformResult result = queryTransform(target_frame, source_frame);
        return toTransformMatrix(result);
    }
    
    // 静态方法：从变换结果生成变换矩阵 - 使用 float
    static Eigen::Matrix4f toTransformMatrix(const TransformResult& result) {
        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        matrix.block<3, 3>(0, 0) = result.rotation.toRotationMatrix();
        matrix(0, 3) = result.translation.x();
        matrix(1, 3) = result.translation.y();
        matrix(2, 3) = result.translation.z();
        return matrix;
    }
    
    // 静态方法：从矩阵创建变换结果
    static TransformResult fromTransformMatrix(const std::string& parent,
                                              const std::string& child,
                                              const Eigen::Matrix4f& matrix) {
        TransformResult result(parent, child);
        result.translation = matrix.block<3, 1>(0, 3);
        
        Eigen::Matrix3f rot_matrix = matrix.block<3, 3>(0, 0);
        result.rotation = Eigen::Quaternionf(rot_matrix);
        result.euler_angles = result.rotation.toRotationMatrix().eulerAngles(0, 1, 2);
        
        return result;
    }
    
    // 静态方法：从欧拉角创建变换结果
    static TransformResult fromEuler(const std::string& parent,
                                    const std::string& child,
                                    const Eigen::Vector3f& translation,
                                    const Eigen::Vector3f& euler_angles) {
        TransformResult result(parent, child);
        result.translation = translation;
        
        Eigen::AngleAxisf rollAngle(euler_angles.x(), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(euler_angles.y(), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(euler_angles.z(), Eigen::Vector3f::UnitZ());
        
        result.rotation = yawAngle * pitchAngle * rollAngle;
        result.euler_angles = euler_angles;
        
        return result;
    }
    
    // 链式变换：计算从 A 到 C 的变换（通过 B）
    TransformResult chainTransform(const std::string& frame_a,
                                  const std::string& frame_b,
                                  const std::string& frame_c) {
        // 计算 A->B 和 B->C 的变换
        TransformResult ab = queryTransform(frame_a, frame_b);
        TransformResult bc = queryTransform(frame_b, frame_c);
        
        // 组合变换：A->C = A->B * B->C
        Eigen::Matrix4f mat_ab = toTransformMatrix(ab);
        Eigen::Matrix4f mat_bc = toTransformMatrix(bc);
        Eigen::Matrix4f mat_ac = mat_ab * mat_bc;
        
        return fromTransformMatrix(frame_a, frame_c, mat_ac);
    }
    
    // 保存当前 TF 树到 YAML 文件
    bool saveToYAML(const std::string& filepath) {
        try {
            YAML::Emitter out;
            out << YAML::BeginMap;
            
            // 保存变换关系（简化版本）
            for (const auto& frame : known_frames_) {
                auto children = getChildFrames(frame);
                if (!children.empty()) {
                    out << YAML::Key << frame;
                    out << YAML::Value << YAML::BeginMap;
                    
                    for (const auto& child : children) {
                        try {
                            TransformResult tf = queryTransform(frame, child);
                            out << YAML::Key << child;
                            out << YAML::Value << YAML::BeginMap;
                            
                            out << YAML::Key << "x" << YAML::Value << tf.translation.x();
                            out << YAML::Key << "y" << YAML::Value << tf.translation.y();
                            out << YAML::Key << "z" << YAML::Value << tf.translation.z();
                            out << YAML::Key << "roll" << YAML::Value << tf.euler_angles.x();
                            out << YAML::Key << "pitch" << YAML::Value << tf.euler_angles.y();
                            out << YAML::Key << "yaw" << YAML::Value << tf.euler_angles.z();
                            
                            out << YAML::EndMap;
                        } catch (...) {
                            // 跳过无法查询的变换
                        }
                    }
                    
                    out << YAML::EndMap;
                }
            }
            
            out << YAML::EndMap;
            
            std::ofstream fout(filepath);
            if (!fout.is_open()) {
                std::cerr << "Cannot open file: " << filepath << std::endl;
                return false;
            }
            fout << out.c_str();
            fout.close();
            
            std::cout << "Saved transforms to: " << filepath << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Save error: " << e.what() << std::endl;
            return false;
        }
    }
};



class CameraParamsParser {
public:
    /**
     * @brief 从YAML文件加载相机标定参数
     * @param file_path YAML文件路径
     * @return bool 加载是否成功
     */
    bool loadFromFile(const std::string& file_path) {
        try {
            root_node_ = YAML::LoadFile(file_path);
            is_loaded_ = parseAllParameters();
            return is_loaded_;
        } catch (const YAML::Exception& e) {
            std::cerr << "Error loading YAML file: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 从YAML字符串加载相机标定参数
     * @param yaml_str YAML格式字符串
     * @return bool 加载是否成功
     */
    bool loadFromString(const std::string& yaml_str) {
        try {
            root_node_ = YAML::Load(yaml_str);
            is_loaded_ = parseAllParameters();
            return is_loaded_;
        } catch (const YAML::Exception& e) {
            std::cerr << "Error parsing YAML string: " << e.what() << std::endl;
            return false;
        }
    }
    
    // 基础参数访问接口
    int getBoardHeight() const { return board_height_; }
    int getBoardWidth() const { return board_width_; }
    const std::string& getCameraName() const { return camera_name_; }
    int getImageWidth() const { return image_width_; }
    int getImageHeight() const { return image_height_; }
    float getSquareSize() const { return square_size_; }
    const std::string& getDistortionModel() const { return distortion_model_; }
    
    // 矩阵数据访问接口
    const std::vector<std::vector<double>>& getCameraMatrix() const { return camera_matrix_; }
    const std::vector<double>& getDistortionCoefficients() const { return distortion_coefficients_; }
    
    // 获取原始节点（用于访问其他字段）
    const YAML::Node& getRootNode() const { return root_node_; }
    
    // 检查是否已加载
    bool isLoaded() const { return is_loaded_; }
    
    /**
     * @brief 打印关键参数
     */
    void printSummary() const {
        if (!is_loaded_) {
            std::cout << "No calibration data loaded." << std::endl;
            return;
        }
        
        std::cout << "=== Camera Calibration Summary ===" << std::endl;
        std::cout << "Camera: " << camera_name_ << std::endl;
        std::cout << "Image size: " << image_width_ << "x" << image_height_ << std::endl;
        std::cout << "Distortion coefficients count: " << distortion_coefficients_.size() << std::endl;
    }
    
    /**
     * @brief 获取任意字段值
     * @tparam T 返回类型
     * @param key 字段名
     * @return T 字段值，如果不存在返回T的默认值
     */
    template<typename T>
    T get(const std::string& key) const {
        try {
            if (root_node_[key]) {
                return root_node_[key].as<T>();
            }
        } catch (const YAML::Exception& e) {
            std::cerr << "Error getting key '" << key << "': " << e.what() << std::endl;
        }
        return T();
    }
    
    /**
     * @brief 检查字段是否存在
     * @param key 字段名
     * @return bool 是否存在
     */
    bool hasKey(const std::string& key) const {
        return root_node_[key].IsDefined();
    }

private:
    /**
     * @brief 解析所有参数
     * @return bool 解析是否成功
     */
    bool parseAllParameters() {
        try {
            // 解析基础参数
            camera_name_ = root_node_["camera_name"].as<std::string>();
            image_width_ = root_node_["image_width"].as<int>();
            image_height_ = root_node_["image_height"].as<int>();
            
            // 解析相机内参矩阵
            if (!parseMatrix(root_node_["camera_matrix"], camera_matrix_)) {
                std::cerr << "Failed to parse camera_matrix" << std::endl;
                return false;
            }
            
            // 解析畸变系数
            if (!parseVector(root_node_["distortion_coefficients"], distortion_coefficients_)) {
                std::cerr << "Failed to parse distortion_coefficients" << std::endl;
                return false;
            }
            
            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "Error parsing parameters: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 解析矩阵数据
     */
    bool parseMatrix(const YAML::Node& matrix_node, 
                    std::vector<std::vector<double>>& matrix) {
        if (!matrix_node || !matrix_node["data"]) {
            return false;
        }
        
        const auto& data_node = matrix_node["data"];
        int rows = matrix_node["rows"].as<int>();
        int cols = matrix_node["cols"].as<int>();
        
        matrix.resize(rows, std::vector<double>(cols));
        
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                matrix[i][j] = data_node[i * cols + j].as<double>();
            }
        }
        
        return true;
    }
    
    /**
     * @brief 解析向量数据
     */
    bool parseVector(const YAML::Node& vector_node, 
                    std::vector<double>& vector) {
        if (!vector_node || !vector_node["data"]) {
            return false;
        }
        
        const auto& data_node = vector_node["data"];
        int rows = vector_node["rows"].as<int>();
        
        vector.resize(rows);
        for (int i = 0; i < rows; ++i) {
            vector[i] = data_node[i].as<double>();
        }
        
        return true;
    }

private:
    // 基础参数
    int board_height_ = 0;
    int board_width_ = 0;
    std::string camera_name_;
    int image_width_ = 0;
    int image_height_ = 0;
    float square_size_ = 0.0f;
    std::string distortion_model_;
    
    // 矩阵参数
    std::vector<std::vector<double>> camera_matrix_;
    std::vector<double> distortion_coefficients_;
    
    // YAML根节点
    YAML::Node root_node_;
    bool is_loaded_ = false;
};

#endif // 