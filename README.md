# former_robot
Robot packages for Former (https://www.roas.co.kr/former/)

<img width="50%" src="https://www.roas.co.kr/wp-content/uploads/2020/11/FORMER-th.jpg"/>

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
