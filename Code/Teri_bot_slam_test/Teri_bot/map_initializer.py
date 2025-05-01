import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time
from std_msgs.msg import String
from std_srvs.srv import Empty  # 导入Empty服务类型

class MapInitializer(Node):
    """
    发布一个初始空白地图以帮助SLAM初始化
    """
    def __init__(self):
        super().__init__('map_initializer')
        
        # 创建地图发布者 - 直接发布到/map话题
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',  # 直接发布到map话题而不是init_map
            qos_profile=10
        )
        
        # 状态发布者，用于调试
        self.status_pub = self.create_publisher(
            String,
            '/map_initializer_status',
            10
        )
        
        # 定时器，每秒发布一次地图
        self.timer = self.create_timer(0.5, self.publish_map)
        
        # 创建服务，用于禁用地图发布
        self.disable_srv = self.create_service(
            Empty,
            '~/disable_publishing',
            self.disable_publishing_callback
        )
        
        # 创建服务，用于启用地图发布
        self.enable_srv = self.create_service(
            Empty,
            '~/enable_publishing',
            self.enable_publishing_callback
        )
        
        # 地图参数
        self.map_width = 800  # 单位：像素 - 减小尺寸以提高性能
        self.map_height = 800  # 单位：像素
        self.map_resolution = 0.05  # 单位：米/像素
        
        # 统计发布次数
        self.publish_count = 0
        
        # 控制是否发布地图
        self.publishing_enabled = True
        
        self.get_logger().info('地图初始化节点已启动，直接发布到/map话题以帮助SLAM初始化')
        self.get_logger().info('可以通过调用/map_initializer/disable_publishing服务来停止发布')
        self.get_logger().info('可以通过调用/map_initializer/enable_publishing服务来恢复发布')
        
        # 发布状态消息
        self.publish_status("已启动")
    
    def disable_publishing_callback(self, request, response):
        """禁用地图发布的服务回调"""
        if self.publishing_enabled:
            self.publishing_enabled = False
            self.publish_status("地图发布已禁用")
            self.get_logger().info('地图发布已禁用')
        return response
    
    def enable_publishing_callback(self, request, response):
        """启用地图发布的服务回调"""
        if not self.publishing_enabled:
            self.publishing_enabled = True
            self.publish_status("地图发布已启用")
            self.get_logger().info('地图发布已启用')
        return response
    
    def publish_status(self, status_text):
        """发布状态消息"""
        msg = String()
        msg.data = f"地图初始化器状态: {status_text}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)
    
    def publish_map(self):
        """发布一个空白地图"""
        # 如果禁用了发布，则直接返回
        if not self.publishing_enabled:
            return
            
        try:
            map_msg = OccupancyGrid()
            
            # 设置地图元数据
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'map'
            
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width
            map_msg.info.height = self.map_height
            
            # 设置地图原点（左下角）
            map_msg.info.origin.position.x = -self.map_width * self.map_resolution / 2.0
            map_msg.info.origin.position.y = -self.map_height * self.map_resolution / 2.0
            map_msg.info.origin.position.z = 0.0
            
            # 设置原点方向（四元数表示无旋转）
            map_msg.info.origin.orientation.x = 0.0
            map_msg.info.origin.orientation.y = 0.0
            map_msg.info.origin.orientation.z = 0.0
            map_msg.info.origin.orientation.w = 1.0
            
            # 创建地图数据，-1表示未知区域
            map_data = [-1] * (self.map_width * self.map_height)
            
            # 在地图中心设置一个小的已知区域（值为0表示空白区域）
            center_x = self.map_width // 2
            center_y = self.map_height // 2
            radius = 30  # 扩大初始已知区域（像素）
            
            for y in range(center_y - radius, center_y + radius):
                for x in range(center_x - radius, center_x + radius):
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        index = y * self.map_width + x
                        # 计算到中心的距离
                        dist = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                        if dist < radius:
                            map_data[index] = 0  # 空闲区域（可通行）
                        elif dist < radius * 1.2: 
                            # 边缘区域添加一些障碍物，帮助SLAM识别特征
                            if (x + y) % 8 == 0:  # 间隔放置一些障碍物
                                map_data[index] = 100  # 障碍物
            
            # 设置地图数据
            map_msg.data = map_data
            
            # 发布地图
            self.map_pub.publish(map_msg)
            
            # 更新计数并报告
            self.publish_count += 1
            if self.publish_count % 10 == 0:  # 每10次发布报告一次
                self.publish_status(f"已发布地图 {self.publish_count} 次")
            
        except Exception as e:
            self.get_logger().error(f'发布地图时出错: {e}')
            self.publish_status(f"错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_status("由于键盘中断而关闭")
    except Exception as e:
        node.get_logger().error(f'运行时错误: {e}')
        if node:
            node.publish_status(f"致命错误: {e}")
    
    if node:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 