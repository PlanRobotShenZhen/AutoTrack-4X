# DR100 室外导航车

## 环境

- Ubuntu 20.04
- ROS Noetic Ninjemys

## 仿真环境导航

### 建图

启动仿真环境，并录制bag包

```bash
roslaunch dr100 gazebo.launch   # 启动仿真环境
# rosrun rs_to_velodyne rs_to_velodyne XYZI XYZIR    # rs点云转velodyne点云（可选）。 注：雷达仿真包已去除NAN
rosrun rqt_robot_steering rqt_robot_steering    # 控制
rosbag record /rslidar_points /velodyne_points /tf /tf_static /imu/data   # 录bag
```

#### 方式一：SC-LeGO-LOAM

```bash
roslaunch lego_loam run.launch  # 建图
rosbag play .bag --clock
```

结束后自动保存到`~/catkin_ws/maps/LeGO-LOAM`，目录下

#### 方式二：LIO-SAM

```bash
roslaunch lio_sam run.launch  # 建图
rosbag play .bag --clock
```

结束后自动保存到`~/catkin_ws/maps/LIO-SAM`，目录下

注: 建图后可使用`pcl_viewer <file>`查看pcd文件

### 生成代价地图

#### 方式一：pcd2pgm (推荐)

```bash
roslaunch pcd2pgm run.launch

cd /home/plan/catkin_ws/src/mapping/pcd2pgm/pcd2pgm/pcd_map
rosrun map_server map_saver -f finalCloud  # 不要加.pgm
# rosrun map_server map_saver -f GlobalMap  # 不要加.pgm
```
##### 配置说明

1. 直通滤波器：按高度范围（thre_z_min ~ thre_z_max）过滤点云。
    参数：
        thre_z_min=0.0（最低高度）
        thre_z_max=10（最高高度）
        flag_pass_through=0（0=保留范围内，1=保留范围外）
    效果：去除地面/天花板，保留目标高度层。
2. 半径滤波器：剔除半径（thre_radius）内邻近点少于thres_point_count的孤立点。
    参数：
        thre_radius=1（搜索半径，单位：米）
        thres_point_count=3（最小邻近点数）
    效果：去除离散噪声，但可能误删稀疏点云。（当前未启用）
3. 统计学滤波器：基于邻近点平均距离剔除异常值。
    参数：
        MeanK=100（计算平均距离的邻近点数）
        StddevMulThresh=1.0（标准差乘数阈值）
    效果：有效去噪，适合复杂场景，但计算量较大。

4. 参数：
    - **file_directory**: 存放pcd文件的路径，默认为 `$(find pcd2pgm)/pcd_map/`
    - **file_name**: pcd文件名称，默认为 `finalCloud`
    - **thre_z_min**: 选取的范围最小高度
    - **thre_z_max**: 选取的范围最大高度
    - **flag_pass_through**: 选取高度范围内的标志，`0` 表示保留范围内，`1` 表示保留范围外
    - **thre_radius**: 半径滤波的半径
    - **thres_point_count**: 半径滤波的要求点数个数
    - **MeanK**: 统计学滤波的半径
    - **StddevMulThresh**: 统计学滤波的要求点数个数
    - **map_resolution**: 存储的栅格map的分辨率
    - **map_topic_name**: 转换后发布的二维地图的topic
    - **use_passthrough_filter**: 直通滤波开关
    - **use_radius_filter**: 半径滤波开关
    - **use_statistical_filter**: 统计学滤波开关

#### 方式二：octomap

```bash
# pcd -> bt
roslaunch bot_navigation pcd2bt.launch
rosrun octomap_server octomap_saver <Path>finalCloud.bt
# bt -> pgm
roslaunch bot_navigation bt2pgm.launch
rosrun map_server map_saver -f <Path>/finalCloud  # 不要加.pgm
```

### 导航

生成后的代价地图(*.pgm)或需要用GIMP将障碍物的离散点连接成线条

然后将对应地图替换到相应位置：

- 对于LeGO-LOAM：`~/catkin_ws/src/bot_navigation/map/3d/lego/`
- 对于LIO-SAM：`~/catkin_ws/src/bot_navigation/map/3d/lio/`

可配置`~/catkin_ws/src/bot_navigation/param/3d`目录下的导航规划器参数

具体可参考：[ROS::导航参数配置详解](https://blog.csdn.net/weixin_43928944/article/details/119571534)，`rosrun rqt_reconfigure rqt_reconfigure`可动态调整

```bash
roslaunch dr100 gazebo.launch   # 启动仿真环境
# roslaunch dr100 run.launch   # 启动真实环境下的小车驱动与描述

# roslaunch bot_navigation localization.launch # 测试定位

# 启动导航
roslaunch bot_navigation navigation.launch  # 导航
roslaunch bot_navigation rviz.launch  # rivz, 在里面设置goal
```

如果pointcloud_to_laserscan未安装则需要安装：`sudo apt-get install -y ros-noetic-pointcloud-to-laserscan`

如果teb_local_planner未安装则需要安装：`sudo apt-get install -y ros-noetic-teb-local-planner`

## 真实环境导航

**确保已连接以下模块：**

- IMU
- RS-LiDAR-16
- DR100 小车底盘

### 建图

启动真实环境下的小车驱动与描述，并录制bag

```bash
roslaunch dr100 run.launch   # 启动真实环境下的小车驱动与描述
# rosrun rs_to_velodyne rs_to_velodyne XYZI XYZIR    # rs点云转velodyne点云（可选）。 注: 雷达新版驱动需要设置dense_points为true，去除NAN点
rosbag record /rslidar_points /velodyne_points /tf /tf_static /imu/data   # 录bag
```

#### 方式一：SC-LeGO-LOAM

参考 [仿真环境导航->建图->SC-LeGO-LOAM](####方式一：SC-LeGO-LOAM)

#### 方式二：LIO-SAM

参考 [仿真环境导航->建图->LIO-SAM](####方式二：LIO-SAM)

### 生成代价地图

参考 [仿真环境导航->生成代价地图](###生成代价地图)

### 导航

参考 [仿真环境导航->导航](###导航)


## 常用命令

  - 查看topic的frame_id
    
    ```bash
    rostopic echo  </Topic>    | grep   frame_id
    # 激光雷达数据:
    # /velodyne_points------frame_id:"velodyne"--------
    # /rslidar_points------frame_id:"robosense"--------
    # IMU数据:
    # /imu/data-------------frame_id:"imu_base"--------
    # GPS数据(无):
    # /gps/fix--------------frame_id:"navset_link"-----
    ```

  - 清除目标点: `rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}`

  - qt 查看 tf_tree: `rosrun rqt_tf_tree rqt_tf_tree`

  - 生成tf_tree的frames.gv和frames.pdf: `rosrun tf view_frames`

  - git浅拷贝: `git clone --depth 1 --single-branch --branch <branch_name> <url>`
 
## FAQ

1. WSL仿真卡顿，无法调用GPU

  > [Github issue](https://github.com/gazebosim/gz-sim/issues/2595#issuecomment-2337682562) 
  >  
  > 确保Ubuntu20.04, `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 与 `export LIBGL_ALWAYS_SOFTWARE=false`

2. gtsam推荐使用v4.1.0, [gtsam](https://gtsam.org/get_started/)

3. 运行时报错 `error while loading shared libraries: libmetis-gtsam.so: cannot open shared object file: No such file or directory`

  > 解决: `sudo ln -s /usr/local/lib/libmetis-gtsam.so /usr/lib/libmetis-gtsam.so` 或者 `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`

4. rs_to_velodyne 编译报错的问题

  > [Github issue](https://github.com/HViktorTsoi/rs_to_velodyne/issues/13)
  > 
  > (1) C++版本太低，改成 `set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")`
  > (2) `pcl_isnan()`已弃用, 替换为 `std::isnan()`, 且添加头文件`#include <cmath>`

5. 仿真包robosense_simulator发布的点云存在NAN点

  > 修改`GazeboRosRoboSenseLaser.cpp`，加上`#include <cmath>`头文件
  > 
  > 将`if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) ) `这个判断改成
  > ```c++
  > // 检查是否为NAN点，以及其他范围检查
  > if (std::isnan(r) || std::isnan(intensity) || 
  >     (MIN_RANGE >= r) || (r >= MAX_RANGE) || 
  >     (intensity < MIN_INTENSITY)) {
  >     continue;
  > }
  > ```

6. ROS1 不再维护, 设置`export DISABLE_ROS1_EOL_WARNINGS=1`

7. LIO-SAM: `#include <opencv/cv.h>`报错

  > ```c++
  > // #include <opencv/cv.h>
  > #include<opencv2/imgproc.hpp>
  > ```

8. LIO-SAM/LeGO-LOAM的警告

  > 将所有的 uint8_t, uint16_t, uint32_t 替换为: std::uint8_t, std::uint16_t, std::uint32_t
  > 
  > 对于 unused 变量的警告, `[[maybe_unused]] float <变量> = xxx`
  > 

9. LIO-SAM参数

  > [LIO-SAM中extrinsicRot、extQRPY和extrinsicTrans的计算推导](https://blog.csdn.net/zzb714121/article/details/140476885)
  >
  > `extrinsicRot`变量是imu中的六轴(加速度计和陀螺仪)观测量转换到`LiDAR`坐标系的旋转矩阵。
  >
  > `extrinsicRPY`变量是将九轴`imu`中的磁力计坐标系转换到`LiDAR`坐标系。
  >
  > `extrinsicTrans`是`lidar`坐标系的原点在`imu`坐标系下的坐标值。
  >
  > 因此在使用LIO-SAM的时候，如果是`六轴imu`，那么需要将`extrinsicRot`进行配置，`extrinsicRPY`可以直接置为单位阵。如果是`九轴imu`，需要额外配置`extrinsicRPY`。不过值得注意的是，一般情况下磁力计会和`imu`的加速度计和陀螺仪坐标系一致，如果不一致，就需要像LIO-SAM中的作者进行配置了。
  >
  > 另外，以上两个变量作者的标定非常简单，只是物理位置上大致对齐了，然后就计算出了两个旋转矩阵，也可以将`imu`和`LiDAR`进行精细标定。

10. LIO-SAM Z轴漂移过大：考虑调整`z_tollerance`限制Z轴运动

11. LIO-SAM 运行时报 `Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame`

  > gazebo仿真中的控制已经发布了`base_link`到`odom`的TF变换，但是SLAM节点又发布了一次导致冲突
  > 
  > gazebo不发布或者变更LIO-SAM参数:
  > ```yaml
  >     # Frames
  >     lidarFrame: "base_link_est"
  >     baselinkFrame: "base_link_est"
  >     odometryFrame: "odom_est"
  > ```
  > 源自 [CSDN](https://blog.csdn.net/weixin_40599145/article/details/126929222) [Github](https://github.com/linzs-online/robot_gazebo)

12. 导航时 `Could not transform the global plan to the frame of the controller`
  > 局部代价地图里程计坐标系错误local_costmap_params.yaml
    将第二行`global_frame: odom` 改成 `global_frame: map`

13. pcl的问题: pcl的问题主要是需要 C++14的编译
  > CMakeLists.txt: `set(CMAKE_CXX_FLAGS "-std=c++14")`

14. opencv的问题: `error: 'class std::unordered_map<unsigned int, std::vector >' has no member named 'serialize'`
  > 原因：PCl库依赖的flann与Opencv冲突。opencv头文件中的一些宏定义和flann库中的冲突
    解决: `在utility.h`中, 保证pcl库中依赖的flann在opencv头文件之前先包含进去, 即把opencv的头文件放在PCL库之后就解决了

15. opencv的问题: 高版本的ubuntu是OpenCV4
  > 在`utility.h`中的
    `#include<opencv/cv.h>`
    改为
    `#include<opencv2/imgproc.hpp>`

