### Parameters

The teb_local_planner package allows the user to set [Parameters](https://wiki.ros.org/Parameters) in order to customize the behavior. These parameters are grouped into several categories: robot configuration, goal tolerance, trajectory configuration, obstacles, optimization, planning in distinctive topologies and miscellaneous parameters. Some of them are chosen to be compliant with the [base_local_planner](https://wiki.ros.org/base_local_planner). Many (but not all) parameters can be modified at runtime using [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure).

#### Robot Configuration Parameters

`~<name>/acc_lim_x` (`double`, default: 0.5)

- Maximum translational acceleration of the robot in meters/sec^2

`~<name>/acc_lim_theta` (`double`, default: 0.5)

- Maximum angular acceleration of the robot in radians/sec^2

`~<name>/max_vel_x` (`double`, default: 0.4)

- Maximum translational velocity of the robot in meters/sec

`~<name>/max_vel_x_backwards` (`double`, default: 0.2)

- Maximum absolute translational velocity of the robot while driving backwards in meters/sec. See optimization parameter `weight_kinematics_forward_drive`

`~<name>/max_vel_theta` (`double`, default: 0.3)

- Maximum angular velocity of the robot in radians/sec

The following parameters are relevant only for carlike robots:

`~<name>/min_turning_radius` (`double`, default: 0.0)

- Minimum turning radius of a carlike robot (set to zero for a diff-drive robot).

`~<name>/wheelbase` (`double`, default: 1.0)

- The distance between the rear axle and the front axle. The value might be negative for back-wheeled robots (only required if `~<name>/cmd_angle_instead_rotvel`is set to `true`).

`~<name>/cmd_angle_instead_rotvel` (`bool`, default: false)

- Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle [-pi/2,pi/2]. *Note, changing the semantics of yaw rate depending on the application is not preferable. Here, it just complies with the inputs required by the stage simulator. Datatypes in [ackermann_msgs](https://wiki.ros.org/ackermann_msgs) are more appropriate, but are not supported by move_base. The local planner is not intended to send commands by itself.*

The following parameters are relevant only for holonomic robots: **New in ROS kinetic**

**Note**, reduce `~<name>/weight_kinematics_nh` significantly in order to adjust the tradeoff between compliant longitudinal motion and non-compliant lateral motion (strafing).

`~<name>/max_vel_y` (`double`, default: 0.0)

- Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)

`~<name>/acc_lim_y` (`double`, default: 0.5)

- Maximum strafing acceleration of the robot

The following parameters are relevant for the footprint model used for optimization (see [Tutorial Obstacle Avoidance and Robot Footprint Model](https://wiki.ros.org/teb_local_planner/Tutorials/Obstacle Avoidance and Robot Footprint Model)). **New in version 0.3**

`~<name>/footprint_model/type` (`string`, default: "point")

- Specify the robot footprint model type used for optimization. Different types are "point", "circular", "line", "two_circles" and "polygon." The type of the model significantly influences the required computation time.

`~<name>/footprint_model/radius` (`double`, default: 0.2)

- This parameter is only relevant for type "circular". It contains the radius of the circle. The center of the circle is located at the robot's axis of rotation.

`~<name>/footprint_model/line_start` (`double[2]`, default: [-0.3, 0.0])

- This parameter is only relevant for type "line". It contains the start coordinates of the line segment.

`~<name>/footprint_model/line_end` (`double[2]`, default: [0.3, 0.0])

- This parameter is only relevant for type "line". It contains the end coordinates of the line segment.

`~<name>/footprint_model/front_offset` (`double`, default: 0.2)

- This parameter is only relevant for type "two_circles". It describes how much the center of the front circle is shifted along the robot's x-axis. The robot's axis of rotation is assumed to be located at [0,0].

`~<name>/footprint_model/front_radius` (`double`, default: 0.2)

- This parameter is only relevant for type "two_circles". It contains the radius of front circle.

`~<name>/footprint_model/rear_offset` (`double`, default: 0.2)

- This parameter is only relevant for type "two_circles". It describes how much the center of the rear circle is shifted along the robot's negative x-axis. The robot's axis of rotation is assumed to be located at [0,0].

`~<name>/footprint_model/rear_radius` (`double`, default: 0.2)

- This parameter is only relevant for type "two_circles". It contains the radius of rear circle.

`~<name>/footprint_model/vertices` (`double[]`, default: [ [0.25,-0.05], [...], ...])

- This parameter is only relevant for type "polygon". It contains the list of polygon vertices (2d coordinates each). The polygon is always closed: do not repeat the first vertex at the end.

`~<name>/is_footprint_dynamic` (`bool`, default: false)

- If true, updates the footprint before checking trajectory feasibility

#### Goal Tolerance Parameters

`~<name>/xy_goal_tolerance` (`double`, default: 0.2)

- Allowed final euclidean distance to the goal position in meters

`~<name>/yaw_goal_tolerance` (`double`, default: 0.2)

- Allowed final orientation error in radians

`~<name>/free_goal_vel` (`bool`, default: false)

- Remove the goal velocity constraint such that the robot can arrive at the goal with maximum speed

#### Trajectory Configuration Parameters

`~<name>/dt_ref` (`double`, default: 0.3)

- Desired temporal resolution of the trajectory (the trajectory is not fixed to `dt_ref` since the temporal resolution is part of the optimization, but the trajectory will be resized between iterations if *dt_ref +-dt_hysteresis* is violated.

`~<name>/dt_hysteresis` (`double`, default: 0.1)

- Hysteresis for automatic resizing depending on the current temporal resolution, usually approx. 10% of `dt_ref` is recommended

`~<name>/min_samples` (`int`, default: 3)

- Minimum number of samples (should be always greater than 2)

`~<name>/global_plan_overwrite_orientation` (`bool`, default: true)

- Overwrite orientation of local subgoals provided by the global planner (since they often provide only a 2D path)

`~<name>/global_plan_viapoint_sep` (`double`, default: -0.1 (disabled))

- If positive, via-points are extrected from the global plan (path-following mode). The value determines the resolution of the reference path (min. separation between each two consecutive via-points along the global plan, if negative: disabled). Refer to parameter `weight_viapoint` for adjusting the intensity. **New in version 0.4**

`~<name>/max_global_plan_lookahead_dist` (`double`, default: 3.0)

- Specify the maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization. The actual length is than determined by the logical conjunction of the local costmap size and this maximum bound. Set to zero or negative in order to deactivate this limitation.

`~<name>/force_reinit_new_goal_dist` (`double`, default: 1.0)

- Reinitialize the trajectory if a previous goal is updated with a separation of more than the specified value in meters (skip hot-starting)

`~<name>/feasibility_check_no_poses` (`int`, default: 4)

- Specify up to which pose on the predicted plan the feasibility should be checked each sampling interval.

`~<name>/publish_feedback` (`bool`, default: false)

- Publish planner feedback containing the full trajectory and a list of active obstacles (should be enabled only for evaluation or debugging). See list of publishers above.

`~<name>/shrink_horizon_backup` (`bool`, default: true)

- Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues (e.g. infeasibility). Also see parameter `shrink_horizon_min_duration`.

`~<name>/allow_init_with_backwards_motion` (`bool`, default: false)

- If true, underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is only recommended if the robot is equipped with rear sensors).

`~<name>/exact_arc_length` (`bool`, default: false)

- If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations (-> increased cpu time), otherwise the Euclidean approximation is used.

`~<name>/shrink_horizon_min_duration` (`double`, default: 10.0)

- Specify minimum duration for the reduced horizon in case an infeasible trajectory is detected (refer to parameter `shrink_horizon_backup` in order to activate the reduced horizon mode).

#### Obstacle Parameters

`~<name>/min_obstacle_dist` (`double`, default: 0.5)

- Minimum desired separation from obstacles in meters

`~<name>/include_costmap_obstacles` (`bool`, default: true)

- Specify if obstacles of the local costmap should be taken into account. Each cell that is marked as obstacle is considered as a point-obstacle. Therefore do not choose a very small resolution of the costmap since it increases computation time. In future releases this circumstance is going to be addressed as well as providing an additional api for dynamic obstacles.

`~<name>/costmap_obstacles_behind_robot_dist` (`double`, default: 1.0)

- Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters).

`~<name>/obstacle_poses_affected` (`int`, default: 30)

- Each obstacle position is attached to the closest pose on the trajectory in order to keep a distance. Additional neighbors can be taken into account as well. Note, this parameter might be removed in future versions, since the the obstacle association strategy has been modified in kinetic+. Refer to the parameter description of `legacy_obstacle_association`.

`~<name>/inflation_dist` (`double`, default: pre kinetic: 0.0, kinetic+: 0.6)

- Buffer zone around obstacles with non-zero penalty costs (should be larger than `min_obstacle_dist` in order to take effect). Also refer to the weight `weight_inflation`.

`~<name>/include_dynamic_obstacles` (`bool`, default: false)

- If this parameter is set to true, the motion of obstacles with non-zero velocity (provided via user-supplied obstacles on topic `~/obstacles` or obtained from the [costmap_converter](https://wiki.ros.org/costmap_converter)) is predicted and considered during optimization via a constant velocity model. **New**

`~<name>/legacy_obstacle_association` (`bool`, default: false)

- The strategy of connecting trajectory poses with obstacles for optimization has been modified (see changelog). You can switch to the old/previous strategy by setting this parameter to `true`. Old strategy: for each obstacle, find the nearest TEB pose; new strategy: for each teb pose, find only "relevant" obstacles.

`~<name>/obstacle_association_force_inclusion_factor` (`double`, default: 1.5)

- The non-legacy obstacle association strategy tries to connect only relevant obstacles with the discretized trajectory during optimization. But all obstacles within a specifed distance are forced to be included (as a multiple of `min_obstacle_dist`). E.g. choose 2.0 in order to`enforce the consideration obstacles within a radius of 2.0*`min_obstacle_dist. [This parameter is used only if parameter `legacy_obstacle_association` is `false`]

`~<name>/obstacle_association_cutoff_factor` (`double`, default: 5)

- See `obstacle_association_force_inclusion_factor`, but beyond a multiple of [value]*`min_obstacle_dist` all obstacles are ignored during optimization. Parameter `obstacle_association_force_inclusion_factor` is processed first. [This parameter is used only if parameter `legacy_obstacle_association` is `false`]

The following parameters are relevant only if [costmap_converter](https://wiki.ros.org/costmap_converter) plugins are desired (see tutorial):

`~<name>/costmap_converter_plugin` (`string`, default: "")

- Define plugin name in order to convert costmap cells to points/lines/polygons. Set an empty string to disable the conversion such that all cells are treated as point-obstacles.

`~<name>/costmap_converter_spin_thread` (`bool`, default: true)

- If set to true, the costmap converter invokes its callback queue in a different thread.

`~<name>/costmap_converter_rate` (`double`, default: 5.0)

- Rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate) [in Hz].

#### Optimization Parameters

`~<name>/no_inner_iterations` (`int`, default: 5)

- Number of actual solver iterations called in each *outerloop* iteration. See param `no_outer_iterations`.

`~<name>/no_outer_iterations` (`int`, default: 4)

- Each outerloop iteration automatically resizes the trajectory according to the desired temporal resolution `dt_ref` and invokes the internal optimizer (that performs `no_inner_iterations`). The total number of solver iterations in each planning cycle is therefore the product of both values.

`~<name>/penalty_epsilon` (`double`, default: 0.1)

- Add a small safety margin to penalty functions for hard-constraint approximations

`~<name>/weight_max_vel_x` (`double`, default: 2.0)

- Optimization weight for satisfying the maximum allowed translational velocity

`~<name>/weight_max_vel_theta` (`double`, default: 1.0)

- Optimization weight for satisfying the maximum allowed angular velocity

`~<name>/weight_acc_lim_x` (`double`, default: 1.0)

- Optimization weight for satisfying the maximum allowed translational acceleration

`~<name>/weight_acc_lim_theta` (`double`, default: 1.0)

- Optimization weight for satisfying the maximum allowed angular acceleration

`~<name>/weight_kinematics_nh` (`double`, default: 1000.0)

- Optimization weight for satisfying the non-holonomic kinematics (this parameter must be high since the kinematics equation constitutes an equality constraint, even a value of 1000 does not imply a bad matrix condition due to small 'raw' cost values in comparison to other costs).

`~<name>/weight_kinematics_forward_drive` (`double`, default: 1.0)

- Optimization weight for forcing the robot to choose only forward directions (positive transl. velocities). A small weight (e.g. 1.0) still allows driving backwards. A value around 1000 almost prevents backward driving (but cannot be guaranteed).

`~<name>/weight_kinematics_turning_radius` (`double`, default: 1.0)

- Optimization weight for enforcing a minimum turning radius (only for carlike robots).

`~<name>/weight_optimaltime` (`double`, default: 1.0)

- Optimization weight for contracting the trajectory w.r.t transition/execution time

`~<name>/weight_obstacle` (`double`, default: 50.0)

- Optimization weight for keeping a minimum distance from obstacles

`~<name>/weight_viapoint` (`double`, default: 1.0)

- Optimization weight for minimzing the distance to via-points (resp. reference path). **New in version 0.4**

`~<name>/weight_inflation` (`double`, default: 0.1)

- Optimization weight for the inflation penalty (should be small).

`~<name>/weight_adapt_factor` (`double`, default: 2.0)

- Some special weights (currently `weight_obstacle`) are repeatedly scaled by this factor in each outer TEB iteration (weight_new = weight_old*factor). Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.

#### Parallel Planning in distinctive Topologies



`~<name>/enable_homotopy_class_planning` (`bool`, default: true)

- Activate parallel planning in distinctive topologies (requires much more CPU resources, since multiple trajectories are optimized at once)

`~<name>/enable_multithreading` (`bool`, default: true)

- Activate multiple threading in order to plan each trajectory in a different thread

`~<name>/max_number_classes` (`int`, default: 4)

- Specify the maximum number of distinctive trajectories taken into account (limits computational effort)

`~<name>/selection_cost_hysteresis` (`double`, default: 1.0)

- Specify how much trajectory cost must a new candidate have w.r.t. a previously selected trajectory in order to be selected (selection if new_cost < old_cost*factor).

`~<name>/selection_obst_cost_scale` (`double`, default: 100.0)

- Extra scaling of obstacle cost terms just for selecting the 'best' candidate.

`~<name>/selection_viapoint_cost_scale` (`double`, default: 1.0)

- Extra scaling of via-point cost terms just for selecting the 'best' candidate. **New in version 0.4**

`~<name>/selection_alternative_time_cost` (`bool`, default: false)

- If true, time cost (sum of squared time differences) is replaced by the total transition time (sum of time differences).

`~<name>/roadmap_graph_no_samples` (`int`, default: 15)

- Specify the number of samples generated for creating the roadmap graph

`~<name>/roadmap_graph_area_width` (`double`, default: 6)

- Random keypoints/waypoints are sampled in a rectangular region between start and goal. Specify the width of that region in meters.

`~<name>/h_signature_prescaler` (`double`, default: 1.0)

- Scale internal parameter (*H-signature*) that is used to distinguish between homotopy classes. Warning: reduce this parameter only, if you observe problems with too many obstacles in the local cost map, do not choose it extremly low, otherwise obstacles cannot be distinguished from each other (0.2<*value*<=1).

`~<name>/h_signature_threshold` (`double`, default: 0.1)

- Two H-signatures are assumed to be equal, if both the difference of real parts and complex parts are below the specified threshold.

`~<name>/obstacle_heading_threshold` (`double`, default: 1.0)

- Specify the value of the scalar product between obstacle heading and goal heading in order to take them (obstacles) into account for exploration.

`~<name>/visualize_hc_graph` (`bool`, default: false)

- Visualize the graph that is created for exploring distinctive trajectories (check marker message in rviz)

`~<name>/viapoints_all_candidates` (`bool`, default: true)

- If true, all trajectories of different topologies are attached to the set of via-points, otherwise only the trajectory sharing the same topology as the initial/global plan is connected with them (no effect on *test_optim_node*). **New in version 0.4**

`~<name>/switching_blocking_period` (`double`, default: 0.0)

- Specify a time duration in seconds that needs to be expired before a switch to a new equivalence class is allowed.

#### Miscellaneous Parameters

`~<name>/odom_topic` (`string`, default: "odom")

- Topic name of the odometry message, provided by the robot driver or simulator.

`~<name>/map_frame` (`string`, default: "odom")

- Global planning frame (in case of a static map, this parameter must be usually changed to "/map".