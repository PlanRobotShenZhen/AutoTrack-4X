#!/usr/bin/env python3
import serial
import time
import argparse
import sys

def test_baudrate(port, baudrate, timeout=1, duration=5):
    """测试指定波特率下串口是否有数据"""
    print(f"测试波特率: {baudrate} bps...")
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=timeout)
        
        # 清空缓冲区
        ser.reset_input_buffer()
        
        start_time = time.time()
        data_received = False
        bytes_count = 0
        
        # 在指定时间内读取数据
        while time.time() - start_time < duration:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                bytes_count += len(data)
                data_received = True
                print(f"  收到数据: {len(data)} 字节")
                # 打印前20个字节的十六进制表示
                if len(data) > 0:
                    hex_data = ' '.join([f'{b:02X}' for b in data[:20]])
                    print(f"  数据样本: {hex_data}{' ...' if len(data) > 20 else ''}")
            time.sleep(0.1)
        
        ser.close()
        
        if data_received:
            print(f"✓ 波特率 {baudrate} bps 成功接收到 {bytes_count} 字节数据")
            return True
        else:
            print(f"✗ 波特率 {baudrate} bps 未接收到数据")
            return False
            
    except Exception as e:
        print(f"✗ 波特率 {baudrate} bps 测试失败: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(description='串口数据测试工具')
    parser.add_argument('--port', type=str, default='/dev/dr100_classis', help='串口设备路径')
    parser.add_argument('--timeout', type=float, default=1, help='读取超时时间(秒)')
    parser.add_argument('--duration', type=float, default=3, help='每个波特率测试持续时间(秒)')
    parser.add_argument('--baudrates', type=str, default='all', 
                        help='要测试的波特率列表，用逗号分隔，或使用"all"测试所有常用波特率')
    
    args = parser.parse_args()
    
    # 常用波特率列表
    common_baudrates = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    
    # 确定要测试的波特率
    if args.baudrates.lower() == 'all':
        baudrates = common_baudrates
    else:
        try:
            baudrates = [int(b.strip()) for b in args.baudrates.split(',')]
        except ValueError:
            print("错误: 波特率必须是整数")
            sys.exit(1)
    
    print(f"开始测试串口 {args.port}")
    print(f"每个波特率测试持续 {args.duration} 秒")
    print("-" * 50)
    
    success_baudrates = []
    
    for baudrate in baudrates:
        if test_baudrate(args.port, baudrate, args.timeout, args.duration):
            success_baudrates.append(baudrate)
        print("-" * 50)
    
    print("\n测试结果摘要:")
    if success_baudrates:
        print(f"以下波特率成功接收到数据: {', '.join(map(str, success_baudrates))}")
    else:
        print("所有测试的波特率均未接收到数据")

if __name__ == "__main__":
    main()