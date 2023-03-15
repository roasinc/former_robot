# former_bringup

## Install udev rule

```
$ sudo cp 99-former-devices.rules /etc/udev/rules.d/
$ sudo udevadm control --reload
$ sudo udevadm trigger
```

## Install systemd service

For launch the ROS packages on system booting

```
$ sudo cp former_bringup.service /etc/systemd/system/
$ sudo systemctl daemon-reload
$ sudo systemctl enable former_bringup.service
```


### Start/Stop service
```
$ sudo systemctl start former_bringup.service
$ sudo systemctl stop former_bringup.service
```
