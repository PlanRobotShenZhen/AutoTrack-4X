#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from sensor_msgs.msg import Imu
from datetime import datetime

class ImuDataLogger:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('imu_data_logger', anonymous=True)
        
        # 创建CSV文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"imu_data_{timestamp}.csv"
        
        # 打开CSV文件并写入表头
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 
            'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z'
        ])
        
        # 订阅IMU话题
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        rospy.loginfo(f"IMU数据记录器已启动，数据将保存到: {self.csv_filename}")
    
    def imu_callback(self, msg):
        # 获取时间戳
        timestamp = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}"
        
        # 获取线性加速度和角速度
        linear_accel_x = msg.linear_acceleration.x
        linear_accel_y = msg.linear_acceleration.y
        linear_accel_z = msg.linear_acceleration.z
        
        angular_vel_x = msg.angular_velocity.x
        angular_vel_y = msg.angular_velocity.y
        angular_vel_z = msg.angular_velocity.z
        
        # 写入CSV
        self.csv_writer.writerow([
            timestamp,
            linear_accel_x, linear_accel_y, linear_accel_z,
            angular_vel_x, angular_vel_y, angular_vel_z
        ])
        
        # 确保数据立即写入文件
        self.csv_file.flush()
        
    def shutdown(self):
        # 关闭CSV文件
        if self.csv_file:
            self.csv_file.close()
            rospy.loginfo(f"IMU数据已保存到: {self.csv_filename}")

if __name__ == '__main__':
    try:
        logger = ImuDataLogger()
        # 设置关闭时的回调
        rospy.on_shutdown(logger.shutdown)
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        pass