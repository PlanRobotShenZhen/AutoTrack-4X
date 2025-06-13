1. 根据USB供应商ID和产品ID绑定设备端口

```bash
sudo cp ./tools/serial/serial_rules/99-imu.rules /etc/udev/rules.d/
```

2. 重新加载UDEV规则并测试

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔设备，检查是否生成符号链接：

```bash
ls -l /dev/imu_device
```

应显示类似输出：

```
lrwxrwxrwx 1 root root 7 Jan 1 00:00 /dev/imu_device -> ttyUSB0
```

3. 完整命令

```
sudo cp ./tools/serial/serial_rules/* /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout plan
sudo usermod -aG dialout root
# 重启
```