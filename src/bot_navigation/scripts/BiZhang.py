#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs import do_transform_cloud
import threading
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge

class ReactiveObstacleAvoidance:
    def __init__(self):
        rospy.init_node('reactive_obstacle_avoidance')

        # 参数 - 使用更宽松的设置
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/depth/points')
        self.depth_image_topic = rospy.get_param('~depth_image_topic', '/camera/depth/image_raw')
        self.cmd_vel_input_topic = rospy.get_param('~cmd_vel_input', '/move_base/cmd_vel')
        self.cmd_vel_output_topic = rospy.get_param('~cmd_vel_output', '/cmd_vel_safe')
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_depth_optical_frame')
        
        # 更宽松的监控区域参数
        self.grid_range = rospy.get_param('~grid_range', [0.1, 3.0, -1.5, 1.5])  # 扩大范围
        self.grid_resolution = rospy.get_param('~grid_resolution', 0.1)  # 降低分辨率减少计算量
        self.emergency_zone = rospy.get_param('~emergency_zone', [0.1, 1.0, -0.6, 0.6])
        self.slow_zone = rospy.get_param('~slow_zone', [0.1, 2.0, -1.0, 1.0])
        self.clearance_height = rospy.get_param('~clearance_height', 0.15)

        # TF监听
        self.tf_buffer = Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        # 发布/订阅
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_output_topic, Twist, queue_size=1)
        self.debug_pc_pub = rospy.Publisher('/obstacle_avoidance/debug_points', PointCloud2, queue_size=1)
        self.obstacle_grid_pub = rospy.Publisher('/obstacle_avoidance/grid', PointCloud2, queue_size=1)
        
        # 订阅点云和深度图像作为备用
        rospy.Subscriber(self.camera_topic, PointCloud2, self.pointcloud_callback)
        rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback)
        rospy.Subscriber(self.cmd_vel_input_topic, Twist, self.cmd_vel_callback)

        # 状态变量
        self.latest_obstacle_map = None
        self.lock = threading.Lock()
        self.tf_ready = False
        self.pointcloud_count = 0
        self.depth_image_count = 0
        self.last_data_time = rospy.Time.now()
        self.using_depth_image = False

        # 初始化
        self.wait_for_tf()
        
        rospy.loginfo("Reactive Obstacle Avoidance Node Started")
        rospy.loginfo("Monitoring topics: %s and %s", self.camera_topic, self.depth_image_topic)

    def wait_for_tf(self, timeout=15.0):
        """等待必要的TF变换可用"""
        rospy.loginfo("Waiting for TF transform from %s to %s...", self.camera_frame, self.robot_base_frame)
        start_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time) < timeout:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame, 
                    self.camera_frame, 
                    rospy.Time(0),
                    rospy.Duration(2.0)
                )
                self.tf_ready = True
                rospy.loginfo("✓ TF transform available")
                rospy.loginfo("Translation: [%.3f, %.3f, %.3f]", 
                             transform.transform.translation.x,
                             transform.transform.translation.y, 
                             transform.transform.translation.z)
                return
            except TransformException as e:
                rospy.logwarn_throttle(2.0, "Waiting for TF: %s", str(e))
                rospy.sleep(0.5)
        
        rospy.logerr("✗ Timeout waiting for TF transform")

    def depth_image_callback(self, msg):
        """深度图像回调函数 - 作为点云的备用"""
        self.depth_image_count += 1
        self.last_data_time = rospy.Time.now()
        
        if not self.tf_ready:
            return

        try:
            # 将深度图像转换为点云
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            
            # 检查图像是否有效
            if cv_image is None or cv_image.size == 0:
                rospy.logwarn_throttle(5.0, "Received empty depth image")
                return
                
            points = []
            height, width = cv_image.shape
            
            # 简化的相机内参（需要根据实际相机调整）
            fx = 500.0  # 焦距x
            fy = 500.0  # 焦距y
            cx = width / 2.0  # 光学中心x
            cy = height / 2.0  # 光学中心y
            
            # 采样点（避免处理所有点）
            step = 4
            for v in range(0, height, step):
                for u in range(0, width, step):
                    depth = cv_image[v, u]
                    # 过滤无效深度值
                    if depth > 0.1 and depth < 5.0:  # 有效深度范围
                        # 将像素坐标转换为3D坐标
                        x = (u - cx) * depth / fx
                        y = (v - cy) * depth / fy
                        z = depth
                        points.append((x, y, z))
            
            if len(points) > 0:
                self.using_depth_image = True
                self.process_points_from_depth(points, msg.header)
                
        except Exception as e:
            rospy.logwarn("Depth image processing error: %s", e)

    def process_points_from_depth(self, points, header):
        """处理从深度图像转换的点"""
        try:
            x_min, x_max, y_min, y_max = self.grid_range
            x_size = int((x_max - x_min) / self.grid_resolution) + 1
            y_size = int((y_max - y_min) / self.grid_resolution) + 1
            obstacle_grid = np.full((x_size, y_size), -1.0)

            debug_points = []
            valid_point_count = 0

            for point in points:
                x, y, z = point
                
                # 坐标变换（假设相机坐标系与base_link对齐，实际需要TF）
                # 这里简化处理，实际应该使用TF变换
                if (x_min <= z <= x_max and  # 注意：深度相机前方是Z轴
                    y_min <= -x <= y_max and  # 注意坐标轴方向
                    0.1 < z < 3.0):
                    
                    col = int((z - x_min) / self.grid_resolution)  # Z是前方距离
                    row = int((-x - y_min) / self.grid_resolution)  # X是左右距离
                    
                    if 0 <= col < x_size and 0 <= row < y_size:
                        if z > self.clearance_height:
                            obstacle_grid[col, row] = max(obstacle_grid[col, row], z)
                    
                    valid_point_count += 1
                    if z > self.clearance_height and len(debug_points) < 300:
                        debug_points.append((z, -x, 0.1))  # 调整坐标到base_link

            # 发布调试点云
            if debug_points and self.depth_image_count % 5 == 0:
                debug_header = Header()
                debug_header.stamp = rospy.Time.now()
                debug_header.frame_id = self.robot_base_frame
                debug_cloud = create_cloud_xyz32(debug_header, debug_points)
                self.debug_pc_pub.publish(debug_cloud)

            # 更新障碍物地图
            if valid_point_count > 10:
                with self.lock:
                    self.latest_obstacle_map = obstacle_grid
                    
        except Exception as e:
            rospy.logwarn("Points from depth processing error: %s", e)

    def pointcloud_callback(self, msg):
        """处理点云数据"""
        self.pointcloud_count += 1
        self.last_data_time = rospy.Time.now()
        self.using_depth_image = False
        
        # 检查点云是否为空
        if msg.width == 0 or msg.height == 0:
            rospy.logwarn_throttle(10.0, "Received empty point cloud (width=%d, height=%d)", msg.width, msg.height)
            return

        if not self.tf_ready:
            rospy.logwarn_throttle(5.0, "TF not ready, skipping point cloud")
            return

        try:
            # 获取TF变换
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame, 
                msg.header.frame_id, 
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            cloud_in_base = do_transform_cloud(msg, transform)
            self.process_transformed_pointcloud(cloud_in_base, msg.header)
            
        except TransformException as e:
            rospy.logwarn_throttle(2.0, "TF transform failed: %s", e)

    def process_transformed_pointcloud(self, cloud_in_base, original_header):
        """处理已经转换到base_link坐标系的点云"""
        try:
            x_min, x_max, y_min, y_max = self.grid_range
            x_size = int((x_max - x_min) / self.grid_resolution) + 1
            y_size = int((y_max - y_min) / self.grid_resolution) + 1
            obstacle_grid = np.full((x_size, y_size), -1.0)

            debug_points = []
            valid_point_count = 0
            total_points = 0

            # 处理每个点
            for point in read_points(cloud_in_base, field_names=("x", "y", "z"), skip_nans=True):
                total_points += 1
                x, y, z = point
                
                # 更宽松的过滤条件
                if (x_min <= x <= x_max and 
                    y_min <= y <= y_max and 
                    0.05 < z < 5.0):  # 扩大深度范围
                    
                    col = int((x - x_min) / self.grid_resolution)
                    row = int((y - y_min) / self.grid_resolution)
                    
                    if 0 <= col < x_size and 0 <= row < y_size:
                        if z > self.clearance_height:
                            obstacle_grid[col, row] = max(obstacle_grid[col, row], z)
                    
                    valid_point_count += 1
                    if z > self.clearance_height and len(debug_points) < 500:
                        debug_points.append((x, y, 0.1))  # 固定高度便于可视化

            # 发布调试点云
            if debug_points and self.pointcloud_count % 3 == 0:
                debug_header = Header()
                debug_header.stamp = rospy.Time.now()
                debug_header.frame_id = self.robot_base_frame
                debug_cloud = create_cloud_xyz32(debug_header, debug_points)
                self.debug_pc_pub.publish(debug_cloud)

            # 发布障碍物栅格可视化
            if self.pointcloud_count % 10 == 0:
                self.publish_obstacle_grid(obstacle_grid)

            # 定期输出统计信息
            if self.pointcloud_count % 30 == 0:
                rospy.loginfo("PointCloud: %d total, %d valid, %d in range", 
                             total_points, valid_point_count, len(debug_points))

            # 更新障碍物地图
            if valid_point_count > 5:  # 降低阈值
                with self.lock:
                    self.latest_obstacle_map = obstacle_grid
            elif total_points > 0:
                rospy.logwarn_throttle(10.0, "Few valid points: %d/%d (%.1f%%)", 
                                      valid_point_count, total_points, 
                                      (valid_point_count/total_points)*100)
                    
        except Exception as e:
            rospy.logwarn("Point cloud processing error: %s", e)

    def publish_obstacle_grid(self, obstacle_grid):
        """发布障碍物栅格可视化"""
        grid_points = []
        x_min, x_max, y_min, y_max = self.grid_range
        
        for i in range(obstacle_grid.shape[0]):
            for j in range(obstacle_grid.shape[1]):
                if obstacle_grid[i, j] > self.clearance_height:
                    x = x_min + i * self.grid_resolution
                    y = y_min + j * self.grid_resolution
                    z = 0.1  # 固定高度便于可视化
                    grid_points.append((x, y, z))
        
        if grid_points:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.robot_base_frame
            grid_cloud = create_cloud_xyz32(header, grid_points)
            self.obstacle_grid_pub.publish(grid_cloud)

    def cmd_vel_callback(self, msg):
        """处理move_base的cmd_vel，并发布过滤后的安全指令"""
        safe_cmd = Twist()
        safe_cmd.linear = msg.linear
        safe_cmd.angular = msg.angular

        with self.lock:
            current_map = self.latest_obstacle_map

        # 检查数据是否过期
        time_since_data = (rospy.Time.now() - self.last_data_time).to_sec()
        if time_since_data > 2.0:
            rospy.logwarn_throttle(5.0, "No camera data for %.1f seconds", time_since_data)
            # 可以选择停止机器人或者继续前进
            # safe_cmd.linear.x = 0.0
            # safe_cmd.angular.z = 0.0

        # 如果没有障碍物地图，直接转发但记录警告
        if current_map is None:
            if self.pointcloud_count > 5 or self.depth_image_count > 5:
                rospy.logwarn_throttle(10.0, "No obstacle map available - safety disabled")
            self.cmd_vel_pub.publish(safe_cmd)
            return

        # 检查危险区域
        emergency_detected = self.check_zone(current_map, self.emergency_zone)
        slow_detected = self.check_zone(current_map, self.slow_zone)

        # 安全决策
        if emergency_detected:
            rospy.logwarn_throttle(0.5, "EMERGENCY STOP! Obstacle detected")
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        elif slow_detected:
            rospy.loginfo_throttle(2.0, "Obstacle in slow zone. Reducing speed.")
            safe_cmd.linear.x *= 0.3
            safe_cmd.angular.z *= 0.4

        # 发布安全指令
        self.cmd_vel_pub.publish(safe_cmd)

    def check_zone(self, obstacle_map, zone):
        """检查指定区域内是否存在高于阈值的障碍物"""
        x_min, x_max, y_min, y_max = zone
        grid_x_min = int((x_min - self.grid_range[0]) / self.grid_resolution)
        grid_x_max = int((x_max - self.grid_range[0]) / self.grid_resolution)
        grid_y_min = int((y_min - self.grid_range[2]) / self.grid_resolution)
        grid_y_max = int((y_max - self.grid_range[2]) / self.grid_resolution)

        # 确保索引在有效范围内
        grid_x_min = max(0, grid_x_min)
        grid_x_max = min(obstacle_map.shape[0]-1, grid_x_max)
        grid_y_min = max(0, grid_y_min)
        grid_y_max = min(obstacle_map.shape[1]-1, grid_y_max)

        # 检查该区域内的栅格
        obstacle_count = 0
        for i in range(grid_x_min, grid_x_max+1):
            for j in range(grid_y_min, grid_y_max+1):
                if obstacle_map[i, j] > self.clearance_height:
                    obstacle_count += 1
                    if obstacle_count >= 2:  # 至少2个栅格有障碍物才触发
                        return True
        return False

if __name__ == '__main__':
    try:
        node = ReactiveObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
