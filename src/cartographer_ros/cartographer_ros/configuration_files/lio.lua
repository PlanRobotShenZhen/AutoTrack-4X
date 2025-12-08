-- 包含map_builder的配置文件  
include "map_builder.lua"  
-- 包含trajectory_builder的配置文件  
include "trajectory_builder.lua"  
  
-- 主配置选项  
options = {  
  -- 使用从map_builder.lua中定义的MAP_BUILDER配置  
  map_builder = MAP_BUILDER,  
  -- 使用从trajectory_builder.lua中定义的TRAJECTORY_BUILDER配置  
  trajectory_builder = TRAJECTORY_BUILDER,  
  -- 地图的全局参考框架  
  map_frame = "map",  
  -- 跟踪框架，通常设置为机器人的基座或主体框架  
  tracking_frame = "imu_base",    
  -- 发布轨迹的参考框架，通常与tracking_frame相同  
  published_frame = "lidar_link",  
  -- 里程计数据的框架，用于机器人内部的里程计估计  
  odom_frame = "odom",  
  -- 是否由Cartographer提供odom_frame。如果外部系统已提供，则设置为false  
  provide_odom_frame = false,  
  -- 是否将发布的帧投影到2D平面  
  publish_frame_projected_to_2d = false,  
  -- 是否使用姿态外推器来预测机器人位置  
  --use_pose_extrapolator = true,  
  -- 是否使用里程计数据  
  use_odometry = false,  
  -- 是否使用卫星导航系统数据  
  use_nav_sat = false,  
  -- 是否使用地标数据  
  use_landmarks = false,  
  -- 激光扫描传感器的数量  
  num_laser_scans = 0,  
  -- 多回波激光扫描传感器的数量  
  num_multi_echo_laser_scans = 0,  
  -- 每个激光扫描数据的细分数量  
  num_subdivisions_per_laser_scan = 1,  
  -- 点云数据的数量  
  num_point_clouds = 1,  
  -- 查找变换的超时时间（秒）  
  lookup_transform_timeout_sec = 0.2,  
  -- 子图发布的周期（秒）  
  submap_publish_period_sec = 0.3,  
  -- 姿态发布的周期（秒）  
  pose_publish_period_sec = 5e-3,  
  -- 轨迹发布的周期（秒）  
  trajectory_publish_period_sec = 30e-3,  
  -- 测距仪数据的采样比率  
  rangefinder_sampling_ratio = 1.,  
  -- 里程计数据的采样比率  
  odometry_sampling_ratio = 1.,  
  -- 固定框架姿态的采样比率  
  fixed_frame_pose_sampling_ratio = 1.,  
  -- IMU数据的采样比率  
  imu_sampling_ratio = 1.,  
  -- 地标数据的采样比率  
  landmarks_sampling_ratio = 1.,  
}  
  
-- 3D轨迹构建器的特定配置  
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1  
  
-- 地图构建器的特定配置  
MAP_BUILDER.use_trajectory_builder_3d = true  -- 使用3D轨迹构建器  
MAP_BUILDER.num_background_threads = 7        -- 后台处理线程的数量  
  
-- 位姿图优化器的配置  
POSE_GRAPH.optimization_problem.huber_scale = 5e2  -- Huber损失的尺度参数  
POSE_GRAPH.optimize_every_n_nodes = 320           -- 每多少个节点优化一次  
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  -- 约束构建的采样比率  
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10  -- Ceres求解器的最大迭代次数  
POSE_GRAPH.constraint_builder.min_score = 0.62  -- 约束构建的最小得分  
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66  -- 全局定位的最小得分  
  
-- 返回配置选项  
return options
