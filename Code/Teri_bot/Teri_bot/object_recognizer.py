import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool, String

class ObjectRecognizer(Node):
    def __init__(self):
        super().__init__('object_recognizer')
        self.bridge = CvBridge()  # Initialize CvBridge for ROS-to-OpenCV image conversion
        
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,          # Message type
            '/camera1_image', # Topic name to subscribe to
            self.image_callback, # Callback function for processing images
            10              # QoS (queue size)
        )
        
        # Publisher for detected red circle information
        self.detection_publisher = self.create_publisher(
            Bool,
            '/red_circle_detected',
            10
        )
        
        # Publisher for detection details
        self.detection_info_publisher = self.create_publisher(
            String,
            '/red_circle_info',
            10
        )
        
        self.get_logger().info("Object recognizer node has started - will detect red circles and publish to /red_circle_detected topic")

    def image_callback(self, msg):
        """
        Callback function to process received image messages.
        """
        try:
            # Convert ROS 2 image message to OpenCV format (BGR image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert the BGR image to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define the HSV range for red
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            # Create a red mask
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Perform morphological operations to remove noise
            kernel = np.ones((5,5), np.uint8)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours in the red mask
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Default state - no red circle detected
            detected = False
            detection_info = ""
            
            for contour in contours:
                # Calculate the area of the contour
                area = cv2.contourArea(contour)
                if area > 10:  # Ignore small areas
                    # Calculate the perimeter of the contour
                    perimeter = cv2.arcLength(contour, True)
                    # Calculate circularity
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # If circularity is close to 1 (circle), then it's a red circle
                    if 0.7 < circularity < 1.3:
                        # Get the minimum enclosing circle
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        center = (int(x), int(y))
                        radius = int(radius)
                        
                        # Draw the detected red circle on the image
                        cv2.circle(cv_image, center, radius, (0, 255, 0), 2)
                        cv2.putText(cv_image, "Red Circle", (center[0]-50, center[1]-20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Set detection flag and info
                        detected = True
                        detection_info = f"center:({x},{y}),radius:{radius}"
                        
                        # Output detected red circle information
                        self.get_logger().info(f"Detected red circle: center ({x}, {y}), radius {radius}")
                        break
            
            # Publish detection result
            detection_msg = Bool()
            detection_msg.data = detected
            self.detection_publisher.publish(detection_msg)
            
            # Publish detection info if detected
            if detected:
                info_msg = String()
                info_msg.data = detection_info
                self.detection_info_publisher.publish(info_msg)
                self.get_logger().info(f"Published detection status: TRUE and info: {detection_info}")
            else:
                self.get_logger().debug("No red circle detected in this frame")
            
            # Display the processed image
            cv2.imshow("Red Circle Detection", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            # Log any error that occurs during image processing
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the event loop.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = ObjectRecognizer()  # Create an instance of the ObjectRecognizer node

    try:
        rclpy.spin(node)  # Keep the node alive and process callbacks
    except KeyboardInterrupt:
        pass  # Handle graceful shutdown on Ctrl+C

    # Cleanup: destroy the node and close OpenCV windows
    node.destroy_node()  # Destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 client library
    cv2.destroyAllWindows()  # Close all OpenCV windows

if __name__ == '__main__':
    main()  # Entry point of the script

