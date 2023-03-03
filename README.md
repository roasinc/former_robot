# former_robot
Robot packages for Former (https://www.roas.co.kr/former/)

## Specification
- DIMENSIONS (L X W X H): 404 X 404 X 308 MM
- WEIGHT: 35 KG
- GROUND CLEARANCE: 60 MM
- PERFORMANCE
  - MAX. SPEED: 1.5 M/S
  - MAX. PAYLOAD: 70 KG

- TYPE OF DRIVE: TWO WHEELS DIFFERENTIAL DRIVE
- DRIVE POWER: 300 W
- WORKING ENVIRONMENT: INDOOR

<img width="50%" src="https://roas.co.kr/wp-content/uploads/2020/11/FORMER-th.jpg"/>

## Source build

### Get sources

```shell
$ cd ~/dev_ws/src
$ git clone https://github.com/roasinc/former_robot -b humble-devel
```

```shell
$ cd ~/dev_ws/src
$ wstool init
$ wstool merge -y former_robot/requirement.rosinstall
$ wstool update
```

### Install packages for dependencies
```shell
$ cd ~/dev_ws/src
$ rosdep install --from-paths src --ignore-src -r -y
```


### Build

```shell
$ cd ~/dev_ws
$ colcon build --symlink-install
```

### Execute

Gazebo

```shell
$ ros2 launch former_gazebo bringup.launch.py world_name:=office_building.world
```
