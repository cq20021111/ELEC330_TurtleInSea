import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Float64
import math


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create point cloud subscription
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',  # Point cloud topic
            self.point_cloud_callback,
            10
        )

        # Create joint control publishers
        self.publisher_joint2 = self.create_publisher(Float64, 'joint2_move', 10)
        self.publisher_joint3 = self.create_publisher(Float64, 'joint3_move', 10)
        self.publisher_joint4 = self.create_publisher(Float64, 'joint4_move', 10)
        self.publisher_joint5 = self.create_publisher(Float64, 'joint5_move', 10)

        # Create actuator control publishers
        self.publisher_pp2 = self.create_publisher(Float64, '/model/robot1/joint/pp2_joint/cmd_thrust', 10)
        self.publisher_pp3 = self.create_publisher(Float64, '/model/robot1/joint/pp3_joint/cmd_thrust', 10)
        self.publisher_pp4 = self.create_publisher(Float64, '/model/robot1/joint/pp4_joint/cmd_thrust', 10)
        self.publisher_pp5 = self.create_publisher(Float64, '/model/robot1/joint/pp5_joint/cmd_thrust', 10)
        self.publisher_ppt1 = self.create_publisher(Float64, '/model/robot1/joint/ppt1_joint/cmd_thrust', 10)
        self.publisher_ppt2 = self.create_publisher(Float64, '/model/robot1/joint/ppt2_joint/cmd_thrust', 10)

        # Parameters
        self.FORWARD_SPEED = 0.5
        self.BACKWARD_SPEED = -0.65
        self.MIN_DIST_FROM_OBSTACLE = 0.55
        self.ANGLE_VELOCITY = 3.60
        self.base_position = 0.5  # Initial swing position
        self.large_swing = 0.7  # Swing amplitude during turns
        self.rotate = 0  # Rotation state: 0: no rotation, 1: left turn, 2: right turn

        # Robot's front detection area
        self.robot_front_width = 0.48  # Front width (meters)
        self.robot_front_height = 0.15  # Front height (meters)

        # Timer
        self.timer = self.create_timer(0.5, self.timer_callback)

    def point_cloud_callback(self, msg):
        # Parse point cloud data
        points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Initialize minimum distances for left and right
        left_min_dist = float('inf')
        right_min_dist = float('inf')

        # Iterate through point cloud data to compute minimum distances for left and right
        for x, y, z in points:
            if (
                0 < x < self.MIN_DIST_FROM_OBSTACLE and  # Within minimum distance range
                abs(z) < self.robot_front_height / 2  # Within height range
            ):
                if self.robot_front_width / 2 > y > 0:  # Left points
                    left_min_dist = min(left_min_dist, x)
                elif -self.robot_front_width / 2 < y < 0:  # Right points
                    right_min_dist = min(right_min_dist, x)

        # Update rotation state
        if self.rotate == 0:  # No rotation
            if left_min_dist < self.MIN_DIST_FROM_OBSTACLE or right_min_dist < self.MIN_DIST_FROM_OBSTACLE:
                if left_min_dist < right_min_dist:
                    self.rotate = 1  # Update to left turn
                else:
                    self.rotate = 2  # Update to right turn
            else:
                self.set_thrust(forward=True)
        elif self.rotate == 1:  # Left turn
            if left_min_dist >= self.MIN_DIST_FROM_OBSTACLE:
                self.rotate = 0  # Reset to straight
                self.set_thrust(forward=True)
            else:
                self.set_thrust(left=True)
        elif self.rotate == 2:  # Right turn
            if right_min_dist >= self.MIN_DIST_FROM_OBSTACLE:
                self.rotate = 0  # Reset to straight
                self.set_thrust(forward=True)
            else:
                self.set_thrust(right=True)

        # Log state
        self.get_logger().info(f"Rotate state: {self.rotate}, Left min dist: {left_min_dist}, Right min dist: {right_min_dist}")
       
    def set_thrust(self, forward=False, left=False, right=False):
        """Set thrust logic"""
        thrust_pp2 = thrust_pp3 = thrust_pp4 = thrust_pp5 = 0.0
        thrust_ppt1 = thrust_ppt2 = 0.0

        if forward:
            thrust_pp2 = thrust_pp3 = thrust_pp4 = thrust_pp5 = self.FORWARD_SPEED
        elif left:
            thrust_ppt1 = -self.ANGLE_VELOCITY
            thrust_ppt2 = self.ANGLE_VELOCITY
            thrust_pp2 = thrust_pp3 = thrust_pp4 = thrust_pp5 = self.BACKWARD_SPEED
        elif right:
            thrust_ppt1 = self.ANGLE_VELOCITY
            thrust_ppt2 = -self.ANGLE_VELOCITY
            thrust_pp2 = thrust_pp3 = thrust_pp4 = thrust_pp5 = self.BACKWARD_SPEED

        # Publish thrust values
        self.publisher_pp2.publish(Float64(data=thrust_pp2))
        self.publisher_pp3.publish(Float64(data=thrust_pp3))
        self.publisher_pp4.publish(Float64(data=thrust_pp4))
        self.publisher_pp5.publish(Float64(data=thrust_pp5))
        self.publisher_ppt1.publish(Float64(data=thrust_ppt1))
        self.publisher_ppt2.publish(Float64(data=thrust_ppt2))

    def timer_callback(self):
        # Swing logic: adjust joint movements based on rotation state
        self.base_position = -self.base_position
        self.large_swing = -self.large_swing
        if self.rotate == 0:  # No rotation, normal swinging
            position_joint2 = self.base_position
            position_joint3 = -self.base_position
            position_joint4 = -self.base_position
            position_joint5 = self.base_position
        elif self.rotate == 2:  # Right turn, stop left fins, increase right swing
            position_joint2 = 0.0  # Stop left fins
            position_joint3 = 0.0
            position_joint4 = -self.large_swing  # Increase right swing
            position_joint5 = self.large_swing
        elif self.rotate == 1:  # Left turn, stop right fins, increase left swing
            position_joint2 = self.large_swing  # Increase left swing
            position_joint3 = -self.large_swing
            position_joint4 = 0.0  # Stop right fins
            position_joint5 = 0.0

        # Publish joint swing values
        self.publisher_joint2.publish(Float64(data=position_joint2))
        self.publisher_joint3.publish(Float64(data=position_joint3))
        self.publisher_joint4.publish(Float64(data=position_joint4))
        self.publisher_joint5.publish(Float64(data=position_joint5))

        # Log state
        self.get_logger().info(
            f"Joint Positions: joint2: {position_joint2}, joint3: {position_joint3}, "
            f"joint4: {position_joint4}, joint5: {position_joint5}")


def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
