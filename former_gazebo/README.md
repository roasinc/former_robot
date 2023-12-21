# former_gazebo

Gazebo simulation package for former robot

Gazebo version: Gazebo Fortrees

## Usage

```shell
$ ros2 launch former_gazebo bringup.launch.py
```

## Control

```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=base_controller/cmd_vel_unstamped
```

## View on rviz2

```shell
$ rviz2 -d `ros2 pkg prefix former_bringup`/share/former_bringup/view_robot.rviz
```
<center><img width="80%" src="../docs/former_gazebo_rviz.png"/></center>
