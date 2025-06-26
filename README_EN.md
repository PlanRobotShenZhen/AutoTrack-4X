# Outdoor Autonomous SLAM Robot

## License

This project is licensed under the [BSD 3-Clause] License - see the [LICENSE](LICENSE) file for details. This license applies to all historical versions of this project.

## Environment

- Ubuntu 20.04
- ROS Noetic Ninjemys

## Simulation Environment Navigation

### Mapping

Launch simulation environment and record bag files

```bash
roslaunch dr100 gazebo.launch   # Launch simulation environment
# rosrun rs_to_velodyne rs_to_velodyne XYZI XYZIR    # Convert rs point cloud to velodyne point cloud (optional). Note: NAN points removed in radar simulation package
rosrun rqt_robot_steering rqt_robot_steering    # Control
rosbag record /rslidar_points /velodyne_points /tf /tf_static /imu/data   # Record bag
```

#### Method 1: SC-LeGO-LOAM

```bash
roslaunch lego_loam run.launch  # Mapping
rosbag play .bag --clock
```

Automatically saved to `~/catkin_ws/maps/LeGO-LOAM` directory after completion

#### Method 2: LIO-SAM

```bash
roslaunch lio_sam run.launch  # Mapping
rosbag play .bag --clock
```

Automatically saved to `~/catkin_ws/maps/LIO-SAM` directory after completion

Note: After mapping, you can use `pcl_viewer <file>` to view pcd files

### Generate Cost Map

#### Method 1: pcd2pgm (Recommended)

```bash
roslaunch pcd2pgm run.launch

cd /home/plan/catkin_ws/src/mapping/pcd2pgm/pcd2pgm/pcd_map
rosrun map_server map_saver -f finalCloud  # Don't add .pgm
# rosrun map_server map_saver -f GlobalMap  # Don't add .pgm
```
##### Configuration Description

1. Pass-through filter: Filter point cloud by height range (thre_z_min ~ thre_z_max).
    Parameters:
        thre_z_min=0.0 (minimum height)
        thre_z_max=10 (maximum height)
        flag_pass_through=0 (0=keep within range, 1=keep outside range)
    Effect: Remove ground/ceiling, keep target height layer.
2. Radius filter: Remove isolated points with fewer than thres_point_count neighbors within radius (thre_radius).
    Parameters:
        thre_radius=1 (search radius, unit: meters)
        thres_point_count=3 (minimum neighbor count)
    Effect: Remove scattered noise, but may mistakenly delete sparse point clouds. (Currently disabled)
3. Statistical filter: Remove outliers based on average distance of neighboring points.
    Parameters:
        MeanK=100 (number of neighboring points for calculating average distance)
        StddevMulThresh=1.0 (standard deviation multiplier threshold)
    Effect: Effective denoising, suitable for complex scenes, but computationally intensive.

4. Parameters:
    - **file_directory**: Path to store pcd files, default is `$(find pcd2pgm)/pcd_map/`
    - **file_name**: pcd file name, default is `finalCloud`
    - **thre_z_min**: Minimum height of selected range
    - **thre_z_max**: Maximum height of selected range
    - **flag_pass_through**: Flag for selecting height range, `0` means keep within range, `1` means keep outside range
    - **thre_radius**: Radius for radius filtering
    - **thres_point_count**: Required point count for radius filtering
    - **MeanK**: Radius for statistical filtering
    - **StddevMulThresh**: Required point count for statistical filtering
    - **map_resolution**: Resolution of stored grid map
    - **map_topic_name**: Topic of converted 2D map
    - **use_passthrough_filter**: Pass-through filter switch
    - **use_radius_filter**: Radius filter switch
    - **use_statistical_filter**: Statistical filter switch

#### Method 2: octomap

```bash
# pcd -> bt
roslaunch bot_navigation pcd2bt.launch
rosrun octomap_server octomap_saver <Path>finalCloud.bt
# bt -> pgm
roslaunch bot_navigation bt2pgm.launch
rosrun map_server map_saver -f <Path>/finalCloud  # Don't add .pgm
```

### Navigation

The generated cost map (*.pgm) may need GIMP to connect discrete obstacle points into lines

Then replace the corresponding map to the appropriate location:

- For LeGO-LOAM: `~/catkin_ws/src/bot_navigation/map/3d/lego/`
- For LIO-SAM: `~/catkin_ws/src/bot_navigation/map/3d/lio/`

You can configure navigation planner parameters in the `~/catkin_ws/src/bot_navigation/param/3d` directory

For details, refer to: [ROS::Navigation Parameter Configuration](https://blog.csdn.net/weixin_43928944/article/details/119571534), `rosrun rqt_reconfigure rqt_reconfigure` for dynamic adjustment

```bash
roslaunch dr100 gazebo.launch   # Launch simulation environment
# roslaunch dr100 run.launch   # Launch real environment robot driver and description

# roslaunch bot_navigation localization.launch # Test localization

# Launch navigation
roslaunch bot_navigation navigation.launch  # Navigation
roslaunch bot_navigation rviz.launch  # rviz, set goal inside
```

If pointcloud_to_laserscan is not installed, install it: `sudo apt-get install -y ros-noetic-pointcloud-to-laserscan`

If teb_local_planner is not installed, install it: `sudo apt-get install -y ros-noetic-teb-local-planner`

## Real Environment Navigation

**Ensure the following modules are connected:**

- IMU
- RS-LiDAR-16
- DR100 robot chassis

### Mapping

Launch real environment robot driver and description, and record bag files

```bash
roslaunch dr100 run.launch   # Launch real environment robot driver and description
# rosrun rs_to_velodyne rs_to_velodyne XYZI XYZIR    # Convert rs point cloud to velodyne point cloud (optional). Note: New radar driver requires setting dense_points to true to remove NAN points
rosbag record /rslidar_points /velodyne_points /tf /tf_static /imu/data   # Record bag
```

#### Method 1: SC-LeGO-LOAM

Refer to [Simulation Environment Navigation->Mapping->SC-LeGO-LOAM](####Method-1:-SC-LeGO-LOAM)

#### Method 2: LIO-SAM

Refer to [Simulation Environment Navigation->Mapping->LIO-SAM](####Method-2:-LIO-SAM)

### Generate Cost Map

Refer to [Simulation Environment Navigation->Generate Cost Map](###Generate-Cost-Map)

### Navigation

Refer to [Simulation Environment Navigation->Navigation](###Navigation)


## Common Commands

  - View topic frame_id
    
    ```bash
    rostopic echo  </Topic>    | grep   frame_id
    # LiDAR data:
    # /velodyne_points------frame_id:"velodyne"--------
    # /rslidar_points------frame_id:"robosense"--------
    # IMU data:
    # /imu/data-------------frame_id:"imu_base"--------
    # GPS data (none):
    # /gps/fix--------------frame_id:"navset_link"-----
    ```

  - Clear goal point: `rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}`

  - View tf_tree with qt: `rosrun rqt_tf_tree rqt_tf_tree`

  - Generate tf_tree frames.gv and frames.pdf: `rosrun tf view_frames`

  - Git shallow clone: `git clone --depth 1 --single-branch --branch <branch_name> <url>`
 
## FAQ

1. WSL simulation lag, unable to call GPU

  > [Github issue](https://github.com/gazebosim/gz-sim/issues/2595#issuecomment-2337682562) 
  >  
  > Ensure Ubuntu20.04, `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` and `export LIBGL_ALWAYS_SOFTWARE=false`

2. gtsam recommends using v4.1.0, [gtsam](https://gtsam.org/get_started/)

3. Runtime error `error while loading shared libraries: libmetis-gtsam.so: cannot open shared object file: No such file or directory`

  > Solution: `sudo ln -s /usr/local/lib/libmetis-gtsam.so /usr/lib/libmetis-gtsam.so` or `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`

4. rs_to_velodyne compilation error

  > [Github issue](https://github.com/HViktorTsoi/rs_to_velodyne/issues/13)
  > 
  > (1) C++ version too low, change to `set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")`
  > (2) `pcl_isnan()` is deprecated, replace with `std::isnan()`, and add header file `#include <cmath>`

5. Point cloud published by simulation package robosense_simulator contains NAN points

  > Modify `GazeboRosRoboSenseLaser.cpp`, add `#include <cmath>` header file
  > 
  > Change the judgment `if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) )` to
  > ```c++
  > // Check for NAN points and other range checks
  > if (std::isnan(r) || std::isnan(intensity) || 
  >     (MIN_RANGE >= r) || (r >= MAX_RANGE) || 
  >     (intensity < MIN_INTENSITY)) {
  >     continue;
  > }
  > ```

6. ROS1 is no longer maintained, set `export DISABLE_ROS1_EOL_WARNINGS=1`

7. LIO-SAM: `#include <opencv/cv.h>` error

  > ```c++
  > // #include <opencv/cv.h>
  > #include<opencv2/imgproc.hpp>
  > ```

8. LIO-SAM/LeGO-LOAM warnings

  > Replace all uint8_t, uint16_t, uint32_t with: std::uint8_t, std::uint16_t, std::uint32_t
  > 
  > For unused variable warnings, `[[maybe_unused]] float <variable> = xxx`
  > 

9. LIO-SAM parameters

  > [Calculation derivation of extrinsicRot, extQRPY and extrinsicTrans in LIO-SAM](https://blog.csdn.net/zzb714121/article/details/140476885)
  >
  > The `extrinsicRot` variable is the rotation matrix that converts the six-axis (accelerometer and gyroscope) observations in the IMU to the `LiDAR` coordinate system.
  >
  > The `extrinsicRPY` variable converts the magnetometer coordinate system in the nine-axis `IMU` to the `LiDAR` coordinate system.
  >
  > `extrinsicTrans` is the coordinate value of the `lidar` coordinate system origin in the `imu` coordinate system.
  >
  > Therefore, when using LIO-SAM, if it is a `six-axis imu`, you need to configure `extrinsicRot`, and `extrinsicRPY` can be directly set as an identity matrix. If it is a `nine-axis imu`, you need to additionally configure `extrinsicRPY`. It is worth noting that generally the magnetometer will be consistent with the accelerometer and gyroscope coordinate system of the `imu`. If not, you need to configure it like the author in LIO-SAM.
  >
  > In addition, the calibration of the above two variables by the author is very simple, just roughly aligned physically, and then calculated the two rotation matrices. You can also perform fine calibration of `imu` and `LiDAR`.

10. LIO-SAM Z-axis drift too large: Consider adjusting `z_tollerance` to limit Z-axis movement

11. LIO-SAM runtime warning `Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame`

  > The control in gazebo simulation has already published the TF transformation from `base_link` to `odom`, but the SLAM node published it again causing conflict
  > 
  > Either gazebo doesn't publish or change LIO-SAM parameters:
  > ```yaml
  >     # Frames
  >     lidarFrame: "base_link_est"
  >     baselinkFrame: "base_link_est"
  >     odometryFrame: "odom_est"
  > ```
  > Source from [CSDN](https://blog.csdn.net/weixin_40599145/article/details/126929222) [Github](https://github.com/linzs-online/robot_gazebo)

12. Navigation error `Could not transform the global plan to the frame of the controller`
  > Local costmap odometry coordinate system error in local_costmap_params.yaml
    Change line 2 `global_frame: odom` to `global_frame: map`

13. PCL issues: PCL issues mainly require C++14 compilation
  > CMakeLists.txt: `set(CMAKE_CXX_FLAGS "-std=c++14")`

14. OpenCV issue: `error: 'class std::unordered_map<unsigned int, std::vector >' has no member named 'serialize'`
  > Reason: Conflict between flann dependency of PCL library and OpenCV. Some macro definitions in opencv header files conflict with flann library
    Solution: In `utility.h`, ensure that flann dependency in pcl library is included before opencv header files, i.e., put opencv header files after PCL library

15. OpenCV issue: High version ubuntu uses OpenCV4
  > In `utility.h`
    `#include<opencv/cv.h>`
    change to
    `#include<opencv2/imgproc.hpp>`


## Citation

[LIO-SAM (IROS-2020)](https://github.com/TixiaoShan/LIO-SAM)
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
```
@inproceedings{legoloam2018shan,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
