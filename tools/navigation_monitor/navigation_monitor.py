#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
导航监控脚本
1. 监控导航完成状态, 导航完成后向串口发送16进制"01"
2. 监控串口接收控制指令: 01左转, 02右转, 03后退, 04停止, 并发布到/cmd_vel
"""

import rospy
import serial
import threading
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist

STATUS_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST"
}

class Serial:
    def __init__(self):
        self.serial_port = rospy.get_param('~serial_port', '/dev/pts/6')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.linear_speed = rospy.get_param('~linear_speed', 0.01)
        self.angular_speed = rospy.get_param('~angular_speed', 0.02)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.serial_conn = None
        self.serial_lock = threading.Lock()
        
        self.command_map = {
            0x01: "left_turn",
            0x02: "right_turn",
            0x03: "backward",
            0x04: "stop"
        }

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            rospy.loginfo(f"Serial port {self.serial_port} connected successfully")
        except serial.SerialException as e:
            rospy.logerr(f"Serial connection failed: {e}")
            self.serial_conn = None
    
    def send_completion_signal(self):
        """发送导航完成信号(0x01)到串口"""
        with self.serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    data = bytes([0x01])
                    self.serial_conn.write(data)
                    self.serial_conn.flush()
                    rospy.loginfo(f"Serial signal sent successfully, port: {self.serial_port}, data: 0x01")
                except serial.SerialException as e:
                    rospy.logerr(f"Serial send failed: {e}")
            else:
                rospy.logwarn("Serial port not connected, cannot send completion signal")

    def process_serial_command(self, command_byte):
        """处理串口接收到的控制指令"""
        if command_byte in self.command_map:
            command_name = self.command_map[command_byte]
            rospy.loginfo(f"Received serial command: 0x{command_byte:02X} ({command_name})")

            twist = Twist()
            if command_byte == 0x01:  # 左转
                twist.angular.z = self.angular_speed
                rospy.loginfo(f"Turn left: {self.angular_speed}")
            elif command_byte == 0x02:  # 右转
                twist.angular.z = -self.angular_speed
                rospy.loginfo(f"Turn right: {-self.angular_speed}")
            elif command_byte == 0x03:  # 后退
                twist.linear.x = -self.linear_speed
                rospy.loginfo(f"Go backward: {-self.linear_speed}")
            elif command_byte == 0x04:  # 停止
                rospy.loginfo("Car stop")

            self.cmd_vel_pub.publish(twist)
        else:
            rospy.logwarn(f"Unknown command: 0x{command_byte:02X}")

    def serial_receive_thread(self):
        """串口接收线程"""
        rospy.loginfo("Serial receive thread started")
        status_counter = 0
        while not rospy.is_shutdown():
            try:
                with self.serial_lock:
                    if not self.serial_conn or not self.serial_conn.is_open:
                        continue
                    bytes_available = self.serial_conn.in_waiting
                    if bytes_available > 0:
                        data = self.serial_conn.read(1)
                        if not data:
                            continue
                        command_byte = data[0]
                        self.process_serial_command(command_byte)
                    else:
                        status_counter += 1
                        if status_counter >= 50:
                            rospy.loginfo("Waiting for serial data... ")
                            status_counter = 0
                rospy.sleep(0.1)
            except serial.SerialException as e:
                rospy.logerr(f"Serial receive error: {e}")
                rospy.sleep(1)
            except Exception as e:
                rospy.logerr(f"Serial receive thread error: {e}")
                rospy.sleep(1)

        rospy.loginfo("Serial receive thread stopped")


class NavigationMonitor:
    def __init__(self):
        """初始化导航监控器"""
        rospy.init_node('navigation_monitor', anonymous=True)

        self.last_status = None
        self.has_seen_active = False

        self.serial = Serial()
        self.serial.init_serial()

        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        rospy.loginfo("Navigation monitor initialized")

    def status_callback(self, status_array):
        """move_base状态回调函数"""
        if not status_array.status_list:
            return

        # Get the latest status
        latest_status = status_array.status_list[-1]
        current_status = latest_status.status
        goal_id = latest_status.goal_id.id

        if current_status == self.last_status:
            return

        old_name = STATUS_NAMES.get(self.last_status, f"UNKNOWN({self.last_status})")
        new_name = STATUS_NAMES.get(current_status, f"UNKNOWN({current_status})")
        rospy.loginfo(f"Navigation status changed: {old_name} -> {new_name}, Goal ID: {goal_id}")

        if current_status == GoalStatus.ACTIVE:
            self.has_seen_active = True
            rospy.loginfo("Navigation goal is active, start monitoring... ")
        elif current_status == GoalStatus.SUCCEEDED:
            if self.has_seen_active:
                # 只有在之前看到过ACTIVE状态后，SUCCEEDED才触发完成信号, 避免启动时直接是SUCCEEDED的情况
                rospy.loginfo("Navigation goal completed!")
                self.serial.send_completion_signal()
                self.has_seen_active = False
        elif current_status == GoalStatus.ABORTED:
            rospy.logwarn("Navigation goal aborted")
            self.has_seen_active = False
        elif current_status == GoalStatus.PREEMPTED:
            rospy.logwarn("Navigation goal preempted")
            self.has_seen_active = False
        elif current_status == GoalStatus.PENDING:
            rospy.loginfo("Waiting for navigation goal... ")

        self.last_status = current_status

    def run(self):
        """主监控循环"""
        rospy.loginfo("Starting navigation monitor...")

        if self.serial.serial_conn and self.serial.serial_conn.is_open:
            s_r_t = threading.Thread(target=self.serial.serial_receive_thread)
            s_r_t.daemon = True
            s_r_t.start()
            rospy.loginfo("Serial receive thread started")
        else:
            rospy.logwarn("Serial port not connected, skipping serial receive")

        rospy.loginfo("Waiting for navigation goal... ")

        rospy.spin()


def main():
    try:
        monitor = NavigationMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation monitor interrupted")
    except Exception as e:
        rospy.logerr(f"Navigation monitor failed: {e}")


if __name__ == '__main__':
    main()
