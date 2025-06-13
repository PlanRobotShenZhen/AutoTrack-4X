#!/usr/bin/env python3

def calculate_xor_checksum(hex_string):
    """
    计算16进制字符串的异或校验和
    
    参数:
        hex_string: 16进制字符串，如 "7B 00 FF 12 34"
    
    返回:
        (校验和是否正确, 计算得到的校验和, 原始数据中的校验和)
    """
    # 移除所有空格和其他分隔符
    clean_hex = hex_string.replace(" ", "").replace("\n", "").replace("\t", "")
    
    # 确保字符串长度为偶数
    if len(clean_hex) % 2 != 0:
        clean_hex = "0" + clean_hex
    
    # 将16进制字符串转换为字节数组
    bytes_array = bytearray.fromhex(clean_hex)
    
    # 确保至少有2个字节
    if len(bytes_array) < 2:
        raise ValueError("数据太短，至少需要2个字节")
    
    # 获取原始数据中的校验和（倒数第二个字节）
    original_checksum = bytes_array[-2]
    
    # 计算除最后两个字节外所有字节的异或校验和
    calculated_checksum = 0
    for i in range(len(bytes_array) - 2):
        calculated_checksum ^= bytes_array[i]
    
    # 检查校验和是否匹配
    is_valid = (calculated_checksum == original_checksum)
    
    return is_valid, calculated_checksum, original_checksum

# ========================
# 在这里修改参数
# ========================

# 输入的16进制字符串
INPUT_HEX = "7B 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7B 7D"

# ========================
# 主程序
# ========================

def main():
    try:
        # 显示原始数据
        print("\n原始16进制数据:")
        print(INPUT_HEX)
        
        # 计算并验证校验和
        is_valid, calculated_checksum, original_checksum = calculate_xor_checksum(INPUT_HEX)
        
        # 显示结果
        print("\n校验结果:")
        print(f"计算得到的校验和: 0x{calculated_checksum:02X}")
        print(f"原始数据中的校验和: 0x{original_checksum:02X}")
        
        if is_valid:
            print("\n✓ 校验成功: 校验和匹配")
        else:
            print("\n✗ 校验失败: 校验和不匹配")
        
        # 如果校验失败，显示修正后的数据
        if not is_valid:
            # 将16进制字符串转换为字节数组
            clean_hex = INPUT_HEX.replace(" ", "").replace("\n", "").replace("\t", "")
            bytes_array = bytearray.fromhex(clean_hex)
            
            # 修正校验和
            bytes_array[-2] = calculated_checksum
            
            # 将修正后的数据转换回16进制字符串
            corrected_hex = ' '.join([f'{b:02X}' for b in bytes_array])
            
            print("\n修正后的16进制数据:")
            print(corrected_hex)
        
    except Exception as e:
        print(f"错误: {str(e)}")

if __name__ == "__main__":
    main()