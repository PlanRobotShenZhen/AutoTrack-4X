#!/usr/bin/env python3
import serial
import argparse
import time
import sys

class RS232Communicator:
    def __init__(self, port='/dev/ttyS1', baudrate=9600, timeout=1):
        """初始化RS232通信类"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
    
    def connect(self):
        """连接到串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"成功连接到 {self.port}，波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"连接失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"已断开与 {self.port} 的连接")
    
    def send_data(self, data):
        """发送数据"""
        if not self.ser or not self.ser.is_open:
            print("错误: 串口未连接")
            return False
        
        try:
            # 如果输入是字符串，转换为字节
            if isinstance(data, str):
                data = data.encode()
            
            self.ser.write(data)
            print(f"已发送: {' '.join([f'{b:02X}' for b in data])}")
            return True
        except Exception as e:
            print(f"发送失败: {str(e)}")
            return False
    
    def receive_data(self, size=1024):
        """接收数据"""
        if not self.ser or not self.ser.is_open:
            print("错误: 串口未连接")
            return None
        
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(size)
                if data:
                    print(f"已接收: {' '.join([f'{b:02X}' for b in data])}")
                return data
            return None
        except Exception as e:
            print(f"接收失败: {str(e)}")
            return None
    
    def interactive_mode(self):
        """交互模式"""
        print("进入交互模式 (按 Ctrl+C 退出)")
        print("输入格式: 'hex:XX XX XX' 发送十六进制数据")
        print("          'text:Hello' 发送文本数据")
        
        try:
            while True:
                # 检查是否有数据可读
                if self.ser.in_waiting > 0:
                    data = self.receive_data()
                
                # 获取用户输入
                user_input = input("> ")
                
                if user_input.lower().startswith("hex:"):
                    # 处理十六进制输入
                    hex_str = user_input[4:].strip()
                    try:
                        hex_data = bytes.fromhex(hex_str)
                        self.send_data(hex_data)
                    except ValueError:
                        print("错误: 无效的十六进制格式")
                
                elif user_input.lower().startswith("text:"):
                    # 处理文本输入
                    text = user_input[5:]
                    self.send_data(text)
                
                elif user_input.strip():
                    print("错误: 未知的输入格式")
                
                time.sleep(0.1)  # 短暂休眠以减少CPU使用率
                
        except KeyboardInterrupt:
            print("\n退出交互模式")

def main():
    parser = argparse.ArgumentParser(description='RS232串口通信工具')
    parser.add_argument('--port', type=str, default='/dev/ttyS1', help='串口设备路径')
    parser.add_argument('--baudrate', type=int, default=9600, help='波特率')
    parser.add_argument('--timeout', type=float, default=1, help='超时时间(秒)')
    parser.add_argument('--send', type=str, help='要发送的数据(十六进制格式，如"7B 01 7D")')
    parser.add_argument('--interactive', action='store_true', help='启用交互模式')
    
    args = parser.parse_args()
    
    comm = RS232Communicator(args.port, args.baudrate, args.timeout)
    
    if not comm.connect():
        sys.exit(1)
    
    try:
        if args.send:
            try:
                hex_data = bytes.fromhex(args.send)
                comm.send_data(hex_data)
                # 等待响应
                time.sleep(0.5)
                comm.receive_data()
            except ValueError:
                print("错误: 无效的十六进制格式")
        
        elif args.interactive:
            comm.interactive_mode()
        
        else:
            # 默认模式：持续监听
            print("监听模式 (按 Ctrl+C 退出)")
            while True:
                data = comm.receive_data()
                time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n程序已终止")
    finally:
        comm.disconnect()

if __name__ == "__main__":
    main()