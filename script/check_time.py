#!/usr/bin/env python3
"""
ROS 2 Humble – 多话题监听 + 1 Hz 表格输出（清屏版，修复 GNSS/IMU 频率）
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from sensor_msgs.msg import PointCloud2, Image, NavSatFix, Imu
from collections import deque
import threading
import time
import math
import os
import platform

# --------------------- 工具：滑动窗口频率计算器 ---------------------
class FreqCalculator:
    def __init__(self, window_size: int = 30):
        self.timestamps = deque(maxlen=window_size)
        self.lock = threading.Lock()

    def tick(self, t: float):
        with self.lock:
            self.timestamps.append(t)

    def get_freq(self) -> float:
        with self.lock:
            if len(self.timestamps) < 2:
                return 0.0
            return (len(self.timestamps) - 1) / (self.timestamps[-1] - self.timestamps[0])

# --------------------- 主节点 ---------------------
class MultiSubTableNode(Node):
    def __init__(self):
        super().__init__('multi_sub_table')

        self.data = {}
        self.data_lock = threading.Lock()

        topic_map = {
            "/sensing/lidar/front_top/points": PointCloud2,
            "/sensing/lidar/rear_top/points": PointCloud2,
            "/sensing/camera/rear/image_raw": Image,
            "/sensing/camera/rear_3mm/image_raw": Image,
            "/sensing/gnss/fix": NavSatFix,
            "/sensing/imu/imu_data": Imu,
        }

        for topic, msg_type in topic_map.items():
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self.msg_cb(msg, t), 10)
            self.data[topic] = {"stamp": None, "freq": FreqCalculator()}

        self.create_timer(1.0, self.print_table)

    # 统一提取时间戳（优先 header.stamp）
    def extract_stamp(self, msg) -> float:
        if hasattr(msg, 'header'):
            try:
                return Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            except Exception:
                pass
        # 无 header 或解析失败 → 用接收时间
        return self.get_clock().now().nanoseconds * 1e-9

    def msg_cb(self, msg, topic: str):
        stamp = self.extract_stamp(msg)
        with self.data_lock:
            self.data[topic]["stamp"] = stamp
            self.data[topic]["freq"].tick(stamp)

    # --------------------- 打印（清屏） ---------------------
    def print_table(self):
        # 清屏
        if platform.system() == "Windows":
            os.system("cls")
        else:
            print("\033[2J\033[H", end="")

        with self.data_lock:
            base_topic = "/sensing/lidar/front_top/points"
            base_stamp = self.data[base_topic]["stamp"]

            header = "{:<45} | {:>18} | {:>8} | {:>12}".format(
                "Topic", "Latest Stamp (s)", "Freq(Hz)", "Δt w.r.t base(s)")
            print(header)
            print("-" * len(header))

            for topic in self.data:
                stamp = self.data[topic]["stamp"]
                freq = self.data[topic]["freq"].get_freq()
                if stamp is None:
                    stamp_str, delta_str = "None", "N/A"
                else:
                    stamp_str = f"{stamp:.6f}"
                    delta_str = f"{stamp - base_stamp:+.6f}" if base_stamp else "N/A"
                print("{:<45} | {:>18} | {:>8.2f} | {:>12}".format(
                    topic, stamp_str, freq, delta_str))

# --------------------- main ---------------------
def main(args=None):
    rclpy.init(args=args)
    node = MultiSubTableNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
