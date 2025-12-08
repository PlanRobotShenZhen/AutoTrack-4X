#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU数据可视化脚本
读取CSV文件中的IMU数据并绘制曲线图
"""

import pandas as pd
import matplotlib.pyplot as plt

def read_imu_data(csv_file):
    """
    读取IMU数据CSV文件
    
    Args:
        csv_file (str): CSV文件路径
        
    Returns:
        pandas.DataFrame: 包含IMU数据的DataFrame
    """
    try:
        # 读取CSV文件
        df = pd.read_csv(csv_file)
        print(f"成功读取数据，共 {len(df)} 行")
        print(f"数据列: {list(df.columns)}")
        return df
    except Exception as e:
        print(f"读取文件时出错: {e}")
        return None

def plot_imu_data(df):
    """
    绘制IMU数据曲线图
    
    Args:
        df (pandas.DataFrame): IMU数据
    """
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 计算相对时间（从第一个时间戳开始的秒数）
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # 创建子图
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('IMU传感器数据曲线图', fontsize=16, fontweight='bold')
    
    # 线性加速度数据
    axes[0, 0].plot(df['relative_time'], df['linear_accel_x'], 'b-', linewidth=1)
    axes[0, 0].set_title('线性加速度 X轴')
    axes[0, 0].set_xlabel('时间 (秒)')
    axes[0, 0].set_ylabel('加速度 (m/s²)')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(df['relative_time'], df['linear_accel_y'], 'g-', linewidth=1)
    axes[0, 1].set_title('线性加速度 Y轴')
    axes[0, 1].set_xlabel('时间 (秒)')
    axes[0, 1].set_ylabel('加速度 (m/s²)')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[0, 2].plot(df['relative_time'], df['linear_accel_z'], 'r-', linewidth=1)
    axes[0, 2].set_title('线性加速度 Z轴')
    axes[0, 2].set_xlabel('时间 (秒)')
    axes[0, 2].set_ylabel('加速度 (m/s²)')
    axes[0, 2].grid(True, alpha=0.3)
    
    # 角速度数据
    axes[1, 0].plot(df['relative_time'], df['angular_vel_x'], 'c-', linewidth=1)
    axes[1, 0].set_title('角速度 X轴')
    axes[1, 0].set_xlabel('时间 (秒)')
    axes[1, 0].set_ylabel('角速度 (rad/s)')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(df['relative_time'], df['angular_vel_y'], 'm-', linewidth=1)
    axes[1, 1].set_title('角速度 Y轴')
    axes[1, 1].set_xlabel('时间 (秒)')
    axes[1, 1].set_ylabel('角速度 (rad/s)')
    axes[1, 1].grid(True, alpha=0.3)
    
    axes[1, 2].plot(df['relative_time'], df['angular_vel_z'], 'y-', linewidth=1)
    axes[1, 2].set_title('角速度 Z轴')
    axes[1, 2].set_xlabel('时间 (秒)')
    axes[1, 2].set_ylabel('角速度 (rad/s)')
    axes[1, 2].grid(True, alpha=0.3)
    
    # 调整子图间距
    plt.tight_layout()
    
    # 保存图片
    plt.savefig('imu_data_plot.png', dpi=300, bbox_inches='tight')
    print("图表已保存为 'imu_data_plot.png'")
    
    # 显示图表
    plt.show()

def plot_combined_data(df):
    """
    绘制组合数据图表
    
    Args:
        df (pandas.DataFrame): IMU数据
    """
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 计算相对时间
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # 创建组合图表
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('IMU传感器数据组合图', fontsize=16, fontweight='bold')
    
    # 线性加速度组合图
    ax1.plot(df['relative_time'], df['linear_accel_x'], 'b-', label='X轴', linewidth=1)
    ax1.plot(df['relative_time'], df['linear_accel_y'], 'g-', label='Y轴', linewidth=1)
    ax1.plot(df['relative_time'], df['linear_accel_z'], 'r-', label='Z轴', linewidth=1)
    ax1.set_title('线性加速度 (三轴)')
    ax1.set_xlabel('时间 (秒)')
    ax1.set_ylabel('加速度 (m/s²)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 角速度组合图
    ax2.plot(df['relative_time'], df['angular_vel_x'], 'c-', label='X轴', linewidth=1)
    ax2.plot(df['relative_time'], df['angular_vel_y'], 'm-', label='Y轴', linewidth=1)
    ax2.plot(df['relative_time'], df['angular_vel_z'], 'y-', label='Z轴', linewidth=1)
    ax2.set_title('角速度 (三轴)')
    ax2.set_xlabel('时间 (秒)')
    ax2.set_ylabel('角速度 (rad/s)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存组合图
    plt.savefig('imu_combined_plot.png', dpi=300, bbox_inches='tight')
    print("组合图表已保存为 'imu_combined_plot.png'")
    
    plt.show()

def print_data_statistics(df):
    """
    打印数据统计信息
    
    Args:
        df (pandas.DataFrame): IMU数据
    """
    print("\n=== 数据统计信息 ===")
    print(f"数据点数量: {len(df)}")
    print(f"时间跨度: {df['timestamp'].max() - df['timestamp'].min():.3f} 秒")
    
    print("\n线性加速度统计:")
    for axis in ['x', 'y', 'z']:
        col = f'linear_accel_{axis}'
        print(f"  {axis.upper()}轴: 均值={df[col].mean():.6f}, 标准差={df[col].std():.6f}, 范围=[{df[col].min():.6f}, {df[col].max():.6f}]")
    
    print("\n角速度统计:")
    for axis in ['x', 'y', 'z']:
        col = f'angular_vel_{axis}'
        print(f"  {axis.upper()}轴: 均值={df[col].mean():.6f}, 标准差={df[col].std():.6f}, 范围=[{df[col].min():.6f}, {df[col].max():.6f}]")

def plot_individual_data(df):
    """
    为每个传感器数据创建单独的图表

    Args:
        df (pandas.DataFrame): IMU数据
    """
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    # 计算相对时间
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]

    # 传感器数据列表
    sensors = [
        ('linear_accel_x', '线性加速度 X轴', 'm/s²', 'blue'),
        ('linear_accel_y', '线性加速度 Y轴', 'm/s²', 'green'),
        ('linear_accel_z', '线性加速度 Z轴', 'm/s²', 'red'),
        ('angular_vel_x', '角速度 X轴', 'rad/s', 'cyan'),
        ('angular_vel_y', '角速度 Y轴', 'rad/s', 'magenta'),
        ('angular_vel_z', '角速度 Z轴', 'rad/s', 'orange')
    ]

    # 为每个传感器创建单独的图表
    for i, (column, title, unit, color) in enumerate(sensors):
        plt.figure(figsize=(12, 6))

        # 绘制数据
        plt.plot(df['relative_time'], df[column], color=color, linewidth=1.5, alpha=0.8)

        # 设置标题和标签
        plt.title(f'{title} - 详细数据', fontsize=14, fontweight='bold')
        plt.xlabel('时间 (秒)', fontsize=12)
        plt.ylabel(f'{title} ({unit})', fontsize=12)

        # 添加网格
        plt.grid(True, alpha=0.3)

        # 添加统计信息文本框
        mean_val = df[column].mean()
        std_val = df[column].std()
        min_val = df[column].min()
        max_val = df[column].max()

        stats_text = f'统计信息:\n均值: {mean_val:.6f}\n标准差: {std_val:.6f}\n最小值: {min_val:.6f}\n最大值: {max_val:.6f}'
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        # 调整布局
        plt.tight_layout()

        # 保存单独的图表
        filename = f'imu_{column}_individual.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"单独图表已保存: {filename}")

        # 显示图表
        plt.show()

def display_data_values(df, num_samples=20):
    """
    显示数据的具体数值

    Args:
        df (pandas.DataFrame): IMU数据
        num_samples (int): 显示的样本数量
    """
    print(f"\n=== 数据值展示 (前{num_samples}个样本) ===")

    # 计算相对时间
    df_display = df.copy()
    df_display['relative_time'] = df_display['timestamp'] - df_display['timestamp'].iloc[0]

    # 选择要显示的列
    display_columns = ['relative_time', 'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
                      'angular_vel_x', 'angular_vel_y', 'angular_vel_z']

    # 显示前num_samples行数据
    print(df_display[display_columns].head(num_samples).to_string(index=False, float_format='%.6f'))

    if len(df) > num_samples:
        print(f"\n... (还有 {len(df) - num_samples} 行数据)")

    print(f"\n=== 数据值展示 (最后{min(10, len(df))}个样本) ===")
    print(df_display[display_columns].tail(min(10, len(df))).to_string(index=False, float_format='%.6f'))

def export_data_summary(df):
    """
    导出数据摘要到文件

    Args:
        df (pandas.DataFrame): IMU数据
    """
    # 计算相对时间
    df_export = df.copy()
    df_export['relative_time'] = df_export['timestamp'] - df_export['timestamp'].iloc[0]

    # 创建摘要统计
    summary_stats = df_export.describe()

    # 保存到文件
    with open('imu_data_summary.txt', 'w', encoding='utf-8') as f:
        f.write("IMU数据摘要报告\n")
        f.write("=" * 50 + "\n\n")

        f.write(f"数据点总数: {len(df_export)}\n")
        f.write(f"时间跨度: {df_export['relative_time'].max():.3f} 秒\n")
        f.write(f"采样频率: {len(df_export) / df_export['relative_time'].max():.2f} Hz\n\n")

        f.write("详细统计信息:\n")
        f.write("-" * 30 + "\n")
        f.write(summary_stats.to_string())

        f.write("\n\n各传感器数据范围:\n")
        f.write("-" * 30 + "\n")

        sensors = ['linear_accel_x', 'linear_accel_y', 'linear_accel_z',
                  'angular_vel_x', 'angular_vel_y', 'angular_vel_z']

        for sensor in sensors:
            f.write(f"{sensor}:\n")
            f.write(f"  最小值: {df_export[sensor].min():.6f}\n")
            f.write(f"  最大值: {df_export[sensor].max():.6f}\n")
            f.write(f"  均值: {df_export[sensor].mean():.6f}\n")
            f.write(f"  标准差: {df_export[sensor].std():.6f}\n\n")

    print("数据摘要已保存到 'imu_data_summary.txt'")

def main():
    """
    主函数
    """
    # CSV文件路径
    csv_file = 'imu.csv'

    print("开始读取IMU数据...")

    # 读取数据
    df = read_imu_data(csv_file)

    if df is not None:
        # 打印统计信息
        print_data_statistics(df)

        # 显示具体数据值
        display_data_values(df, num_samples=20)

        # 导出数据摘要
        export_data_summary(df)

        # 绘制分离的图表
        print("\n绘制分离图表...")
        plot_imu_data(df)

        # 绘制组合图表
        print("\n绘制组合图表...")
        plot_combined_data(df)

        # 绘制每个传感器的单独图表
        print("\n绘制每个传感器的单独图表...")
        plot_individual_data(df)

        print("\n数据可视化完成！")
        print("生成的文件:")
        print("- imu_data_plot.png (分离图表)")
        print("- imu_combined_plot.png (组合图表)")
        print("- imu_*_individual.png (6个单独图表)")
        print("- imu_data_summary.txt (数据摘要)")

    else:
        print("无法读取数据，程序退出。")

if __name__ == "__main__":
    main()
