import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        # Subscribe to simulated camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.image_callback,
            10
        )
# Publisher for processed image
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.cv_bridge = CvBridge()
    def image_callback(self, msg):
# Convert ROS image to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
# Convert to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
# Define color ranges
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
# Create masks
        red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
# Find contours
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Draw contours on original image
        cv2.drawContours(cv_image, red_contours, -1, (0, 255, 0), 2)
# Publish processed image
        processed_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.publisher.publish(processed_msg)

        if len(red_contours) > 0:
                self.get_logger().info(f'Red detected! Number of red objects: {len(red_contours)}')
        else:
                self.get_logger().info('No red objects detected.')
    
# Log detected objects
        self.get_logger().info(f'Red Objects: {len(red_contours)}')
    def main(args=None):
        rclpy.init(args=args)
        node = ColorDetectionNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    if __name__ == '__main__':
        main()