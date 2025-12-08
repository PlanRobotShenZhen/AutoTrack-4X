# 导航监控

导航监控脚本，用于监控导航状态和接收串口控制指令

## 使用方法

1. 链接串口

如果使用真实串口设备，需要将串口设备链接到小车上

如果使用虚拟串口对进行测试，需要先启动虚拟串口对

```bash
sudo python3 ~/catkin_ws/tools/navigation_monitor/test_navigation_monitor.py
```

然后给予用户串口的读写权限。以`/dev/pts/6`为例：

```bash
sudo chmod 777 /dev/pts/6
```

并在`navigation_monitor.py`中修改串口参数

2. 启动导航监控脚本

```bash
sudo python3 ~/catkin_ws/tools/navigation_monitor/navigation_monitor.py
```

3. 测试导航监控脚本 - 发送导航状态

先发送导航进行状态，再发送导航成功状态

```bash
# 发送导航进行状态
rostopic pub /move_base/status actionlib_msgs/GoalStatusArray "status_list: [{status: 1}]"

# 发送导航成功状态
rostopic pub /move_base/status actionlib_msgs/GoalStatusArray "status_list: [{status: 3}]"
```

之后可以在`test_navigation_monitor.py`中看到导航监控脚本发送的`0x01`

4. 测试导航监控脚本 - 发送控制指令

在`test_navigation_monitor.py`中发送控制指令，可以看到导航监控脚本对串口数据做出相应的速度控制
