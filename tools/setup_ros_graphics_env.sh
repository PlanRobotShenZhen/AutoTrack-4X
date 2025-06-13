#!/bin/bash

# 定义要添加的内容
content='
# export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
# export LIBGL_ALWAYS_SOFTWARE=false
# 在WSL2 Ubuntu 22.04 使用软件渲染后rviz才能正常显示小车模型
# export LIBGL_ALWAYS_SOFTWARE=true
export DISABLE_ROS1_EOL_WARNINGS=1

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
'

# 检查.bashrc文件中是否已存在这些内容
if grep -q "MESA_D3D12_DEFAULT_ADAPTER_NAME" ~/.bashrc; then
    echo "环境变量已经存在于.bashrc文件中，无需重复添加。"
else
    # 将内容追加到.bashrc文件
    echo "$content" >> ~/.bashrc
    echo "环境变量已成功添加到.bashrc文件中。"
    
    # 提示用户可能需要重新加载.bashrc
    echo "请运行以下命令使更改生效："
    echo "source ~/.bashrc"
fi
