#!/usr/bin/env python3
import serial
import time
import argparse
import sys
import os
from datetime import datetime

def log_serial_data(port, baudrate, log_file, max_bytes_per_line=20, max_lines=1000, timeout=1):
    """连接串口并将接收到的数据以16进制格式写入日志文件"""
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"成功连接到 {port}，波特率: {baudrate} bps")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        
        # 创建或打开日志文件
        with open(log_file, 'w') as f:
            f.write(f"# 串口数据日志 - 端口: {port} - 波特率: {baudrate} bps\n")
            f.write(f"# 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 格式: [时间戳] 十六进制数据\n\n")
            
            line_count = 0
            
            try:
                while line_count < max_lines:
                    # 检查是否有数据可读
                    if ser.in_waiting > 0:
                        # 读取数据
                        data = ser.read(ser.in_waiting)
                        if data:
                            # 转换为16进制格式
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            
                            # 确保只显示指定数量的字节
                            display_data = data[:max_bytes_per_line]
                            hex_data = ' '.join([f'{b:02X}' for b in display_data])
                            
                            # 写入日志文件
                            log_line = f"[{timestamp}] {hex_data}"
                            if len(data) > max_bytes_per_line:
                                log_line += f" ... (收到的数据共 {len(data)} 字节)"
                            
                            f.write(log_line + "\n")
                            f.flush()  # 确保数据立即写入文件
                            
                            # 在控制台显示
                            print(log_line)
                            
                            line_count += 1
                    
                    time.sleep(0.1)  # 短暂休眠以减少CPU使用率
                
                print(f"\n已达到最大行数限制 ({max_lines} 行)，记录停止")
                
            except KeyboardInterrupt:
                print("\n用户中断，停止记录")
            
            f.write(f"\n# 结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 总记录行数: {line_count}\n")
            
        ser.close()
        print(f"串口已关闭，日志已保存到 {log_file}")
        
    except Exception as e:
        print(f"错误: {str(e)}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='串口数据记录工具')
    parser.add_argument('--port', type=str, required=True, help='串口设备路径，例如 /dev/ttyS0')
    parser.add_argument('--baudrate', type=int, required=True, help='波特率，例如 9600 或 115200')
    parser.add_argument('--log', type=str, default='serial_log.txt', help='日志文件路径')
    parser.add_argument('--max-bytes', type=int, default=50, help='每行最大字节数')
    parser.add_argument('--max-lines', type=int, default=1000, help='最大记录行数')
    parser.add_argument('--timeout', type=float, default=1, help='读取超时时间(秒)')
    
    args = parser.parse_args()
    
    print(f"开始记录串口 {args.port} 的数据，波特率: {args.baudrate} bps")
    print(f"日志将保存到: {args.log}")
    print(f"每行最多显示 {args.max_bytes} 字节，最多记录 {args.max_lines} 行")
    print("按 Ctrl+C 停止记录")
    print("-" * 50)
    
    log_serial_data(
        args.port, 
        args.baudrate, 
        args.log, 
        args.max_bytes, 
        args.max_lines, 
        args.timeout
    )

if __name__ == "__main__":
    main()
