import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class MirrorSubscriber(Node):
    def __init__(self):
        super().__init__('mirror_viewer_node')
        self.bridge = CvBridge()

        # 订阅前向摄像头
        self.front_sub = self.create_subscription(
            CompressedImage,
            '/electronic_rearview_mirror/front_3mm/camera_image_jpeg',
            self.front_callback,
            10)

        # 订阅后向摄像头
        self.rear_sub = self.create_subscription(
            CompressedImage,
            '/electronic_rearview_mirror/rear_3mm/camera_image_jpeg',
            self.rear_callback,
            10)

        self.get_logger().info('电子后视镜订阅节点已启动，等待图像数据...')

    def process_and_show(self, msg, window_name):
        try:
            # 将压缩图像转换为 OpenCV 格式 (BGR)
            # 使用 cv_bridge 可以自动处理，或者手动使用 cv2.imdecode
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 显示图像
            cv2.imshow(window_name, cv_image)
            cv2.waitKey(1)  # 刷新窗口
        except Exception as e:
            self.get_logger().error(f'图像解码失败 ({window_name}): {str(e)}')

    def front_callback(self, msg):
        self.process_and_show(msg, 'Front Mirror (3mm)')

    def rear_callback(self, msg):
        self.process_and_show(msg, 'Rear Mirror (3mm)')

def main(args=None):
    rclpy.init(args=args)
    node = MirrorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('正在关闭节点...')
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
