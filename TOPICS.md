# Base Controller

- /base_controller/cmd_vel_unstamped [sensor_msgs/msg/Twist.msg]
- /base_controller/odom [sensor_msgs/msg/Odometery.msg]
- /joint_states [sensor_msgs/msg/JointState.msg

# Former GPIO Controller

- /former_io_controller/enable_motor [std_msgs/msg/Bool.msg]
- /former_io_controller/robot_feedback [former_interfaces/msg/RobotFeedback.msg]
- /dock_state [std_msgs/msg/Bool.msg]
- /l_sonar_range [sensor_msgs/msg/Range.msg]
- /r_sonar_range [sensor_msgs/msg/Range.msg]

# D435 Camera

- /camera/color/camera_info [sensor_msgs/msg/CameraInfo.msg]
- /camera/color/image_raw [sensor_msgs/msg/Image.msg]
- /camera/depth/camera_info [sensor_msgs/msg/CameraInfo.msg]
- /camera/depth/color/points [sensor_msgs/msg/PointCloud2.msg]
- /camera/depth/image_rect_raw [sensor_msgs/msg/Image.msg]
- /camera/imu [sensor_msgs/msg/Imu.msg]

# TIM571

- /front_lidar/encoder [x]
- /front_lidar/imu [x]
- /cloud [x]
- /scan [sensor_msgs/msg/Scan.msg]
- /diagnostics

# IMU (XG6000)
- /imu_raw [sensor_msgs/msg/Imu.msg]

# Joystick
- /joy
- /joy/set_feedback

# Etc
- /line_markers
- /robot_description
- /tf
- /tf_static
