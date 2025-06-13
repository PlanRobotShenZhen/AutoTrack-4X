

### 安装依赖

```bash
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y  libpcap-dev
sudo apt-get install -y libqt5serialport5-dev
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
sudo apt-get install -y ros-noetic-navigation*
sudo apt-get install -y ros-noetic-pointcloud-to-laserscan
sudo apt-get install -y ros-noetic-teb-local-planner
sudo apt-get install -y ros-noetic-serial
```

### 编译

可以使用`catkin_make clean`先测试检查是否缺少依赖

```bash
# 1. 单独编译 lego_loam 的消息文件cloud_msgs (单线程)
catkin_make --pkg cloud_msgs -j1

# 2. 编译剩余所有包 (多线程)
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j4 -DCMAKE_BUILD_TYPE=Release
```


## 端口设置

```
sudo cp ./tools/serial/serial_rules/* /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout plan
sudo usermod -aG dialout root
# 重启
```