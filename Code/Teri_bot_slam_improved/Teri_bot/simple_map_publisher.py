import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class SimpleMapPublisher(Node):
    def __init__(self):
        super().__init__('simple_map_publisher')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Simple map publisher started')

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = 0.05
        map_msg.info.width = 200
        map_msg.info.height = 200
        
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        
        map_msg.info.origin.orientation.w = 1.0
        
        # Create a simple empty map
        map_data = [-1] * (map_msg.info.width * map_msg.info.height)
        
        # Add a small square in the center
        center_x = map_msg.info.width // 2
        center_y = map_msg.info.height // 2
        for y in range(center_y-10, center_y+10):
            for x in range(center_x-10, center_x+10):
                idx = y * map_msg.info.width + x
                map_data[idx] = 0  # Free space
        
        map_msg.data = map_data
        self.map_pub.publish(map_msg)
        self.get_logger().info('Published map')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main( )