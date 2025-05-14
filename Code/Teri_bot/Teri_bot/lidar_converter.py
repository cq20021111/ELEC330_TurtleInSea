import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py.point_cloud2 import read_points
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

class LidarConverter(Node):
    """
    ROS2 node
    """
    def __init__(self):
        super().__init__('lidar_converter')
        
        # Add parameters for TF buffer management
        self.declare_parameter('tf_buffer_duration', 2.0)  # Buffer duration in seconds
        self.declare_parameter('transform_tolerance', 1.0)  # Transform tolerance
        
        # Get parameter values
        self.tf_buffer_duration = self.get_parameter('tf_buffer_duration').get_parameter_value().double_value
        self.transform_tolerance = self.get_parameter('transform_tolerance').get_parameter_value().double_value
        
   
        self.horizontal_fov = 360.0  
        self.angle_increment = 0.5  
        self.min_height = -0.2     
        self.max_height = 0.2       
        self.min_range = 0.05      
        self.max_range = 15.0      
      
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',  
            self.point_cloud_callback,
            10
        )
        
   
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',  
        )
        
        # Set up TF buffer with specified duration
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=self.tf_buffer_duration))
        
    
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # odom到base_link的TF - reduce frequency to avoid queue overflow
        self.odom_tf_timer = self.create_timer(0.5, self.publish_odom_tf)  # Changed from 0.1 to 0.5
        
        # Add timer for additional TF broadcasts
        self.laser_map_tf_timer = self.create_timer(0.5, self.publish_laser_map_tf)
        self.base_map_tf_timer = self.create_timer(0.5, self.publish_base_map_tf)
    
    def point_cloud_callback(self, msg):
        """LaserScan"""
        try:
            # LaserScan
            scan_msg = LaserScan()
            scan_msg.header = msg.header
            scan_msg.header.frame_id = "laser"
            
            # LaserScan
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = math.radians(self.horizontal_fov) - math.radians(self.angle_increment)
            scan_msg.angle_increment = math.radians(self.angle_increment)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = self.min_range
            scan_msg.range_max = self.max_range
            
 
            num_points = int(self.horizontal_fov / self.angle_increment)
            scan_msg.ranges = [float('inf')] * num_points
            

            points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            point_count = 0  
            
     
            for x, y, z in points:
             
                if self.min_height <= z <= self.max_height:
                  
                    distance = math.sqrt(x*x + y*y)
                    
              
                    angle = math.degrees(math.atan2(y, x))
                    if angle < 0:
                        angle += 360.0
                    
                   
                    index = int(angle / self.angle_increment)
                    if 0 <= index < num_points:
                      
                        if distance < scan_msg.ranges[index]:
                            scan_msg.ranges[index] = distance
                            point_count += 1
            
            # Apply simple filtering to smooth the scan data
            filtered_ranges = self.filter_scan_data(scan_msg.ranges)
            scan_msg.ranges = filtered_ranges
            
          
            if point_count > 10:  
                self.scan_pub.publish(scan_msg)
                
               
                self.publish_tf_transform()
                
         
                if hasattr(self, 'scan_count'):
                    self.scan_count += 1
                    if self.scan_count % 10 == 0:
                        self.get_logger().info(f': {point_count}')
                else:
                    self.scan_count = 1
                    self.get_logger().info(f': {point_count}')
            else:
                self.get_logger().warn(f' ({point_count}),   ')
            
        except Exception as e:
            self.get_logger().error(f'cloud wrong: {e}')
    
    def filter_scan_data(self, ranges):
        """Apply filtering to improve scan quality"""
        filtered_ranges = list(ranges)  # Make a copy
        window_size = 3
        
        # Apply median filter
        for i in range(len(ranges)):
            # Get values within window
            window = []
            for j in range(-window_size//2, window_size//2 + 1):
                idx = (i + j) % len(ranges)
                val = ranges[idx]
                if val < self.max_range:
                    window.append(val)
            
            # If window has values, use median
            if window:
                filtered_ranges[i] = sorted(window)[len(window)//2]
        
        return filtered_ranges
    
    def publish_tf_transform(self):
        """Publish the base_link to laser transform with adjusted timestamp"""
        t = TransformStamped()
        
        # Get current time and add tolerance for future-dating
        current_time = self.get_clock().now()
        t.header.stamp = current_time.to_msg()
        
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.15
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odom_tf(self):
        """  """
        t = TransformStamped()
        
        # Use current time with tolerance
        current_time = self.get_clock().now()
        t.header.stamp = current_time.to_msg()
        
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        

        self.tf_broadcaster.sendTransform(t)
    
    def publish_laser_map_tf(self):
        """Publish direct laser to map transform to ensure path in TF tree"""
        t = TransformStamped()
        
        # Use current time
        current_time = self.get_clock().now()
        t.header.stamp = current_time.to_msg()
        
        t.header.frame_id = 'map'
        t.child_frame_id = 'laser'
        
        # Identity transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_base_map_tf(self):
        """Publish direct base_link to map transform to ensure path in TF tree"""
        t = TransformStamped()
        
        # Use current time
        current_time = self.get_clock().now()
        t.header.stamp = current_time.to_msg()
        
        t.header.frame_id = 'map' 
        t.child_frame_id = 'base_link'
        
        # Identity transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LidarConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()