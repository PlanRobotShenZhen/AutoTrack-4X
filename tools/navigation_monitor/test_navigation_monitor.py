#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口控制指令测试脚本
使用模拟串口测试navigation_monitor的接收功能
支持虚拟串口对和真实串口设备
"""

import serial
import time
import sys
import os
import subprocess
import threading

class VirtualSerialTester:
    def __init__(self, use_virtual=True):
        self.use_virtual = use_virtual
        self.ser = None
        self.virtual_ports = None
        self.receive_thread = None
        self.stop_receiving = False

        # 控制指令定义
        self.commands = {
            '1': (0x01, "左转"),
            '2': (0x02, "右转"),
            '3': (0x03, "后退"),
            '4': (0x04, "停止")
        }

    def create_virtual_serial_pair(self):
        """创建虚拟串口对"""
        try:
            # 使用socat创建虚拟串口对
            print("正在创建虚拟串口对...")

            # 检查socat是否安装
            result = subprocess.run(['which', 'socat'], capture_output=True)
            if result.returncode != 0:
                print("错误: 未找到socat工具")
                print("请安装: sudo apt-get install socat")
                return None

            # 创建虚拟串口对
            cmd = ['socat', '-d', '-d', 'pty,raw,echo=0', 'pty,raw,echo=0']

            # 启动socat进程
            self.socat_process = subprocess.Popen(
                cmd,
                stderr=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True
            )

            # 等待socat启动并读取输出
            ports = []
            timeout = 10  # 10秒超时
            start_time = time.time()
            stderr_lines = []

            while len(ports) < 2 and (time.time() - start_time) < timeout:
                # 检查进程是否还在运行
                if self.socat_process.poll() is not None:
                    print("socat进程意外退出")
                    # 读取剩余的stderr输出
                    remaining_output = self.socat_process.stderr.read()
                    if remaining_output:
                        stderr_lines.extend(remaining_output.splitlines())
                    break

                # 非阻塞读取stderr
                try:
                    time.sleep(0.1)
                    # 尝试读取一行
                    try:
                        line = self.socat_process.stderr.readline()
                        if line:
                            stderr_lines.append(line.strip())
                            print(f"socat输出: {line.strip()}")

                            if '/dev/pts/' in line:
                                import re
                                match = re.search(r'/dev/pts/\d+', line)
                                if match:
                                    port = match.group()
                                    if port and port not in ports:
                                        ports.append(port)
                                        print(f"发现虚拟串口: {port}")
                    except:
                        pass
                except Exception as e:
                    print(f"读取socat输出时出错: {e}")
                    time.sleep(0.1)

            if len(ports) >= 2:
                self.virtual_ports = (ports[0], ports[1])
                print(f"虚拟串口对创建成功:")
                print(f"  测试端: {ports[0]}")
                print(f"  监控端: {ports[1]}")
                print(f"请在navigation_monitor.launch中设置: serial_port:={ports[1]}")
                return ports[0]  # 返回测试端口
            else:
                print(f"错误: 超时或无法解析虚拟串口路径，只找到 {len(ports)} 个端口")
                if ports:
                    print(f"找到的端口: {ports}")
                if stderr_lines:
                    print("完整的socat输出:")
                    for line in stderr_lines:
                        print(f"  {line}")
                return None

        except Exception as e:
            print(f"创建虚拟串口对失败: {e}")
            return None

    def connect_serial(self, port, baud_rate=9600):
        """连接串口"""
        try:
            print(f"正在连接串口 {port}，波特率 {baud_rate}...")
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            print("串口连接成功！")

            # 启动数据接收线程
            self.start_receive_thread()
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    def receive_data_thread(self):
        """数据接收线程"""
        print("开始监听串口数据...")
        while not self.stop_receiving and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # 读取可用数据
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        # 显示接收到的数据
                        hex_data = ' '.join([f'0x{b:02X}' for b in data])
                        ascii_data = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data])
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"\n[{timestamp}] 接收到数据:")
                        print(f"  十六进制: {hex_data}")
                        print(f"  ASCII: {ascii_data}")
                        print(">> ", end='', flush=True)  # 重新显示输入提示符
                else:
                    time.sleep(0.1)
            except Exception as e:
                if not self.stop_receiving:
                    print(f"\n数据接收错误: {e}")
                break

    def start_receive_thread(self):
        """启动数据接收线程"""
        if self.receive_thread is None or not self.receive_thread.is_alive():
            self.stop_receiving = False
            self.receive_thread = threading.Thread(target=self.receive_data_thread, daemon=True)
            self.receive_thread.start()

    def stop_receive_thread(self):
        """停止数据接收线程"""
        self.stop_receiving = True
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2)

    def send_command(self, cmd_key):
        """发送控制指令"""
        if cmd_key in self.commands and self.ser:
            cmd_byte, cmd_name = self.commands[cmd_key]
            try:
                self.ser.write(bytes([cmd_byte]))
                self.ser.flush()
                print(f"已发送: {cmd_name} (0x{cmd_byte:02X})")
                return True
            except Exception as e:
                print(f"发送指令失败: {e}")
                return False
        return False

    def auto_test_sequence(self):
        """自动测试序列"""
        print("\n开始自动测试序列...")
        test_sequence = [
            ('1', 4),  # 左转2秒
            ('4', 1),  # 停止1秒
            ('2', 4),  # 右转2秒
            ('4', 1),  # 停止1秒
            ('3', 4),  # 后退2秒
            ('4', 1),  # 停止1秒
        ]

        for cmd, duration in test_sequence:
            if self.send_command(cmd):
                print(f"等待 {duration} 秒...")
                time.sleep(duration)
            else:
                print("自动测试中断")
                break

        print("自动测试序列完成")

    def interactive_test(self):
        """交互式测试"""
        print("\n控制指令:")
        print("1 - 左转 (0x01)")
        print("2 - 右转 (0x02)")
        print("3 - 后退 (0x03)")
        print("4 - 停止 (0x04)")
        print("a - 自动测试序列")
        print("r - 显示接收数据状态")
        print("q - 退出")
        print("\n请输入指令:")
        print("注意: 接收到的数据会实时显示在下方")

        while True:
            try:
                user_input = input(">> ").strip().lower()

                if user_input == 'q':
                    print("退出测试")
                    break
                elif user_input == 'a':
                    self.auto_test_sequence()
                elif user_input == 'r':
                    self.show_receive_status()
                elif user_input in self.commands:
                    self.send_command(user_input)
                else:
                    print("无效输入，请输入 1-4, a, r 或 q")

            except KeyboardInterrupt:
                print("\n\n收到中断信号，退出测试")
                break
            except Exception as e:
                print(f"测试过程中出错: {e}")

    def show_receive_status(self):
        """显示接收状态"""
        if self.receive_thread and self.receive_thread.is_alive():
            print("数据接收线程: 运行中")
        else:
            print("数据接收线程: 未运行")

        if self.ser and self.ser.is_open:
            print(f"串口状态: 已连接 ({self.ser.port})")
            print(f"缓冲区数据: {self.ser.in_waiting} 字节")
        else:
            print("串口状态: 未连接")

    def cleanup(self):
        """清理资源"""
        # 停止数据接收线程
        self.stop_receive_thread()

        if self.ser:
            self.ser.close()
            print("串口已关闭")

        if hasattr(self, 'socat_process'):
            try:
                self.socat_process.terminate()
                self.socat_process.wait(timeout=5)
                print("虚拟串口对已关闭")
            except:
                self.socat_process.kill()

    def run_test(self, port=None, baud_rate=9600):
        """运行测试"""
        try:
            if self.use_virtual and not port:
                # 创建虚拟串口对
                port = self.create_virtual_serial_pair()
                if not port:
                    print("无法创建虚拟串口，退出测试")
                    return
            elif not port:
                port = '/dev/ttyV1'

            # 连接串口
            if self.connect_serial(port, baud_rate):
                # 开始交互式测试
                self.interactive_test()

        finally:
            self.cleanup()

def main():
    """主函数"""
    use_virtual = True
    port = None
    baud_rate = 9600

    # 解析命令行参数
    if len(sys.argv) > 1:
        if sys.argv[1] == '--real':
            use_virtual = False
            if len(sys.argv) > 2:
                port = sys.argv[2]
            if len(sys.argv) > 3:
                baud_rate = int(sys.argv[3])
        else:
            port = sys.argv[1]
            use_virtual = False
            if len(sys.argv) > 2:
                baud_rate = int(sys.argv[2])

    print("=== 串口控制指令测试 ===")
    if use_virtual:
        print("模式: 虚拟串口对")
        print("注意: 请在另一个终端启动navigation_monitor并使用显示的监控端口")
    else:
        print(f"模式: 真实串口")
        print(f"串口: {port or '/dev/ttyV1'}")
    print(f"波特率: {baud_rate}")
    print("=" * 40)

    tester = VirtualSerialTester(use_virtual)
    tester.run_test(port, baud_rate)

if __name__ == '__main__':
    main()
