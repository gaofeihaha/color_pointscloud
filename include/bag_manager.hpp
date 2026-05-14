// bag_manager.hpp - 修复版本
#ifndef BAG_MANAGER_HPP_
#define BAG_MANAGER_HPP_

#include <memory>
#include <string>
#include <set>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <unordered_set>
#include <functional>
#include <rclcpp/time.hpp>
#include <rclcpp/serialization.hpp>
#include "rosbag2_cpp/writer.hpp"
#include <rcpputils/filesystem_helper.hpp>
#include <queue>
#include <deque>
#include <chrono>
#include <iostream>

class BagManager
{
public:
  explicit BagManager(const std::string & bag_file_path, size_t max_queue_size = 10000, const std::string & folder_name = "")
    : bag_file_name_(bag_file_path)
    , max_queue_size_(max_queue_size)
    , stop_requested_(false)
    , message_count_(0)
    , written_count_(0)
  {
    std::string time_folder = folder_name;
    if (time_folder.empty()) {
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
      time_folder = ss.str();
    }

    // 2. 组合最终路径：bag_file_path / 20251012_103012
    rcpputils::fs::path root(bag_file_path);
    rcpputils::fs::path dir = root / time_folder;

    /* 3. 仅当目录名 == time_folder 时才允许清空 */
    if (!rcpputils::fs::exists(root)) 
    {
      std::cout << "BagManager: Created directory " << root.string() << std::endl;
      rcpputils::fs::create_directories(root);
    }
    bag_file_name_ = dir.string();

    // 初始化writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions sopt;
    sopt.uri = bag_file_name_;
    sopt.storage_id = "mcap";
    sopt.storage_preset_profile = "zstd_fast"; // 开启 zstd 压缩

    rosbag2_cpp::ConverterOptions copt;
    copt.input_serialization_format = "cdr";
    copt.output_serialization_format = "cdr";

    writer_->open(sopt, copt);
    
    // 启动写入线程
    writing_thread_ = std::thread(&BagManager::writingThread, this);
  }
  
  ~BagManager()
  {
    stop_record();
  }

  BagManager(const BagManager &) = delete;
  BagManager & operator=(const BagManager &) = delete;

  void start_record()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    recording_ = true;
    cv_.notify_all();
    
    std::cout << "BagManager: Started recording" << std::endl;
  }
  
  void stop_record()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stop_requested_) {
        return;
      }
      stop_requested_ = true;
      recording_ = false;
    }
    
    cv_.notify_all();
    
    if (writing_thread_.joinable()) {
      writing_thread_.join();
    }
    
    // 确保所有队列中的消息都被写入
    flushQueueToFile();
    
    if (writer_) {
      writer_->close();
      writer_.reset();
    }
    
    std::cout << "BagManager: Stopped recording. ";
    std::cout << "Messages: added=" << message_count_.load() 
              << ", written=" << written_count_.load() << std::endl;
  }
  
  template <typename MsgT>
  bool addMessage(const MsgT & msg, const std::string & topic, const rclcpp::Time & stamp)
  {
    // 检查是否应该接收消息
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stop_requested_ || !recording_) {
        return false;
      }
    }
    
    message_count_++;
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // 队列满时的处理策略
    if (message_queue_.size() >= max_queue_size_) {
      // 可以选择丢弃最旧的消息（FIFO）或丢弃当前消息
      // 这里我们丢弃最旧的消息
      message_queue_.pop_front();
    }
    
    // 创建写入函数
    auto write_func = [this, msg, topic, stamp]() {
      try {
        ensureTopicRegistered<MsgT>(topic);
        writer_->write(msg, topic, stamp);
        written_count_++;
      } catch (const std::exception& e) {
        std::cerr << "Write failed: " << e.what() << std::endl;
      }
    };
    
    // 按时间戳插入到正确位置（保持时间顺序）
    auto it = message_queue_.begin();
    while (it != message_queue_.end() && it->timestamp < stamp) {
      ++it;
    }
    
    message_queue_.insert(it, {write_func, stamp});
    
    lock.unlock();
    cv_.notify_one();
    
    return true;
  }
  
  template <typename MsgT>
  bool write(const MsgT & msg, const std::string & topic, const rclcpp::Time & stamp)
  {
    if (!writer_ || stop_requested_) {
      return false;
    }
    
    try {
      ensureTopicRegistered<MsgT>(topic);
      writer_->write(msg, topic, stamp);
      written_count_++;
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Direct write failed: " << e.what() << std::endl;
      return false;
    }
  }
  
  size_t queue_size() const
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return message_queue_.size();
  }
  
  size_t max_queue_size() const { return max_queue_size_; }
  
  void set_max_queue_size(size_t size) 
  { 
    max_queue_size_ = size; 
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // 如果新大小小于当前队列大小，丢弃最旧的消息
    while (message_queue_.size() > max_queue_size_) {
      message_queue_.pop_front();
    }
  }
  
  bool is_recording() const 
  { 
    std::lock_guard<std::mutex> lock(state_mutex_);
    return recording_; 
  }
  
  size_t get_message_count() const { return message_count_.load(); }
  size_t get_written_count() const { return written_count_.load(); }

private:
  struct QueuedMessage {
    std::function<void()> write_func;
    rclcpp::Time timestamp;
    
    // 用于排序的比较函数
    bool operator<(const QueuedMessage& other) const {
      return timestamp < other.timestamp;
    }
  };
  
  template <typename MsgT>
  void ensureTopicRegistered(const std::string& topic)
  {
    std::lock_guard<std::mutex> lock(topic_mutex_);
    
    if (registered_topics_.find(topic) == registered_topics_.end()) {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = topic;
      topic_metadata.type = rosidl_generator_traits::name<MsgT>();
      topic_metadata.serialization_format = "cdr";
      
      writer_->create_topic(topic_metadata);
      registered_topics_.insert(topic);
      
      std::cout << "Registered topic: " << topic << std::endl;
    }
  }
  
  void flushQueueToFile()
  {
    std::deque<QueuedMessage> temp_queue;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      temp_queue.swap(message_queue_);
    }
    
    std::cout << "Flushing " << temp_queue.size() << " messages to file" << std::endl;
    
    for (auto& queued_msg : temp_queue) {
      if (queued_msg.write_func) {
        try {
          queued_msg.write_func();
        } catch (const std::exception& e) {
          std::cerr << "Flush write failed: " << e.what() << std::endl;
        }
      }
    }
  }
  
  void writingThread()
  {
    std::cout << "Writing thread started" << std::endl;
    
    while (true) {
      std::deque<QueuedMessage> batch;
      
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // 等待条件：有消息可写且正在录制，或者停止请求
        cv_.wait(lock, [this]() {
          return !message_queue_.empty() || stop_requested_;
        });
        
        // 检查停止请求
        if (stop_requested_ && message_queue_.empty()) {
          break;
        }
        
        // 批量获取消息（最多100个，避免长时间持有锁）
        size_t batch_size = std::min(message_queue_.size(), size_t(100));
        for (size_t i = 0; i < batch_size; ++i) {
          batch.push_back(std::move(message_queue_.front()));
          message_queue_.pop_front();
        }
      }
      
      // 写入文件
      for (auto& queued_msg : batch) {
        if (queued_msg.write_func) {
          try {
            queued_msg.write_func();
          } catch (const std::exception& e) {
            std::cerr << "Thread write failed: " << e.what() << std::endl;
          }
        }
      }
    }
    
    std::cout << "Writing thread stopped" << std::endl;
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string bag_file_name_;
  
  // 消息队列（按时间戳排序）
  std::deque<QueuedMessage> message_queue_;
  mutable std::mutex queue_mutex_;
  
  std::thread writing_thread_;
  
  // Topic管理
  mutable std::mutex topic_mutex_;
  std::unordered_set<std::string> registered_topics_;
  
  // 统计
  size_t max_queue_size_;
  std::atomic<size_t> message_count_;
  std::atomic<size_t> written_count_;

  // 状态控制
  mutable std::mutex state_mutex_;
  std::condition_variable cv_;
  bool recording_ = false;
  bool stop_requested_ = false;
};

#endif  // BAG_MANAGER_HPP_