# former_robot
Robot packages for Former (https://www.roas.co.kr/former/)

<img width="50%" src="https://www.roas.co.kr/wp-content/uploads/2020/11/FORMER-th.jpg"/>

## Source build

### Get sources

```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/roasinc/former_robot -b noetic-devel
```

### Install packages for dependencies
```shell
$ cd ~/catkin_ws/src
$ rosdep install --from-paths former_robot --ignore-src -r -y
```

### Build

```shell
$ cd ~/catkin_ws
$ catkin build
```