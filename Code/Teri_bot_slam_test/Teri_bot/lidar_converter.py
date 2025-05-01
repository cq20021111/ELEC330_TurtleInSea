import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py.point_cloud2 import read_points
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class LidarConverter(Node):
    """
    ROS2节点，将3D点云数据转换为2D激光扫描数据以用于SLAM
    """
    def __init__(self):
        super().__init__('lidar_converter')
        
        # 配置参数 - 调整以获得更好的激光数据
        self.horizontal_fov = 360.0  # 水平扫描角度范围（度）
        self.angle_increment = 0.5   # 分辨率（度）- 更小的值获得更多点
        self.min_height = -0.2      # 增大高度范围以包含更多点
        self.max_height = 0.2       # 增大高度范围以包含更多点
        self.min_range = 0.05       # 减小最小范围
        self.max_range = 15.0       # 增大最大范围
        
        # 创建点云订阅者
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',  # 点云话题
            self.point_cloud_callback,
            10
        )
        
        # 创建LaserScan发布者
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',  # 标准激光扫描话题
            10
        )
        
        # 创建TF广播器，用于发布base_link到laser变换
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 添加odom到base_link的TF
        self.odom_tf_timer = self.create_timer(0.1, self.publish_odom_tf)
        
        self.get_logger().info('Lidar转换节点已启动 - 将3D点云转换为2D LaserScan')
    
    def point_cloud_callback(self, msg):
        """处理接收到的点云数据并转换为LaserScan"""
        try:
            # 准备LaserScan消息
            scan_msg = LaserScan()
            scan_msg.header = msg.header
            scan_msg.header.frame_id = "laser"
            
            # 设置LaserScan参数
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = math.radians(self.horizontal_fov) - math.radians(self.angle_increment)
            scan_msg.angle_increment = math.radians(self.angle_increment)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = self.min_range
            scan_msg.range_max = self.max_range
            
            # 计算激光点数
            num_points = int(self.horizontal_fov / self.angle_increment)
            scan_msg.ranges = [float('inf')] * num_points
            
            # 从点云中读取点
            points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            point_count = 0  # 计数有效点的数量
            
            # 处理每个点，提取2D扫描数据
            for x, y, z in points:
                # 只考虑特定高度范围内的点
                if self.min_height <= z <= self.max_height:
                    # 计算水平距离和角度
                    distance = math.sqrt(x*x + y*y)
                    
                    # # 跳过无效距离的点
                    # if distance < self.min_range or distance > self.max_range:
                    #     continue
                    
                    # 计算角度（0-360度）
                    angle = math.degrees(math.atan2(y, x))
                    if angle < 0:
                        angle += 360.0
                    
                    # 找到对应的激光扫描点索引
                    index = int(angle / self.angle_increment)
                    if 0 <= index < num_points:
                        # 保存最近的距离
                        if distance < scan_msg.ranges[index]:
                            scan_msg.ranges[index] = distance
                            point_count += 1
            
            # 只有当有足够的有效点时才发布扫描结果
            if point_count > 10:  # 至少需要10个有效点
                # 发布LaserScan消息
                self.scan_pub.publish(scan_msg)
                
                # 发布TF变换 (base_link -> laser)
                self.publish_tf_transform()
                
                # 每10次扫描输出一次信息
                if hasattr(self, 'scan_count'):
                    self.scan_count += 1
                    if self.scan_count % 10 == 0:
                        self.get_logger().info(f'发布激光扫描数据，有效点数: {point_count}')
                else:
                    self.scan_count = 1
                    self.get_logger().info(f'发布激光扫描数据，有效点数: {point_count}')
            else:
                self.get_logger().warn(f'点云中有效点太少 ({point_count}), 跳过发布激光扫描')
            
        except Exception as e:
            self.get_logger().error(f'处理点云数据时出错: {e}')
    
    def publish_tf_transform(self):
        """发布base_link到laser的TF变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        
        # 设置变换（激光雷达相对于机器人基座的位置和方向）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.15  # 激光雷达高度
        
        # 四元数表示无旋转
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # 发布变换
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odom_tf(self):
        """发布odom到base_link的TF变换，以防TF树不完整"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 简单的恒等变换
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # 发布变换
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