#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import LoopClosure
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import threading

class SlamContinuousUpdater(Node):
    def __init__(self):
        super().__init__('slam_continuous_updater')
        
        # Service client for manual loop closure
        self.loop_closure_client = self.create_client(
            LoopClosure, '/slam_toolbox/manual_loop_closure')
        
        # Subscribe to scan to know when new data arrives
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Subscribe to map to monitor updates
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Counters
        self.scan_count = 0
        self.map_update_count = 0
        self.last_map_update_at_scan = 0
        
        # Timer to force updates
        self.timer = self.create_timer(2.0, self.force_update_callback)
        
        self.get_logger().info('SLAM Continuous Updater started')
    
    def scan_callback(self, msg):
        self.scan_count += 1
        
        # Force update every 10 scans if map hasn't updated
        if self.scan_count % 10 == 0:
            if self.scan_count - self.last_map_update_at_scan > 10:
                self.get_logger().info(f'Forcing update at scan {self.scan_count}')
                self.force_slam_update()
    
    def map_callback(self, msg):
        self.map_update_count += 1
        self.last_map_update_at_scan = self.scan_count
        self.get_logger().info(f'Map updated (update #{self.map_update_count} at scan {self.scan_count})')
    
    def force_update_callback(self):
        # Check if we need to force an update
        scans_since_update = self.scan_count - self.last_map_update_at_scan
        if scans_since_update > 20:  # If no update in 20 scans
            self.get_logger().warning(f'No map update in {scans_since_update} scans, forcing update')
            self.force_slam_update()
    
    def force_slam_update(self):
        if self.loop_closure_client.wait_for_service(timeout_sec=1.0):
            request = LoopClosure.Request()
            future = self.loop_closure_client.call_async(request)
            future.add_done_callback(self.update_done_callback)
    
    def update_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Forced SLAM update completed')
        except Exception as e:
            self.get_logger().error(f'Update failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    updater = SlamContinuousUpdater()
    rclpy.spin(updater)
    updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()