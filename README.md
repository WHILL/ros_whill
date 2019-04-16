# ros_whill

<img src="https://user-images.githubusercontent.com/2618822/44189349-e4f39800-a15d-11e8-9261-79edac310e6a.png" width="100px">

ros_whill is a ROS package for [WHILL Model CR](https://whill.jp/model-cr).
We also have [a FAQ and developers community website](https://whill.zendesk.com/hc/ja) for current and potential Model CR users. For general questions and requests, please visit https://whill.zendesk.com/hc/ja .

## ROS API

### Subscribed Topics

#### /whill/controller/joy [(sensor_msgs/Joy)](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
- Virtual WHILL joystick input. You can controll WHILL via this topic.


### Published Topics

#### /whill/states/joy [(sensor_msgs/Joy)](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
- Joystick status

#### /whill/states/jointState [(sensor_msgs/JointState)](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
- Wheel rotate position(rad) and rotation velocity(rad/s)

#### /whill/states/imu [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- IMU measured data.

#### /whill/states/batteryState [(sensor_msgs/BatteryState)](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
- Battery information


## Requirements
- Ubuntu 16.04
- ROS kinetic

## Build
In your shell:
```sh
cd ~/catkin_ws
catkin_make
rospack profile
```
ros_whill package is sometimes not recognized if you not set "rospack profile". (After executed "rosrun ros_whill", you might see the error message "not found package".)

### Build only ros_whill package
```sh
catkin_make -DCATKIN_WHITELIST_PACKAGES="ros_whill"
```

## SerialPort Settings

### Set

Edit your `~/.bashrc` (bash) or `~/.zshrc` (zsh) to add this line:

```sh
export TTY_WHILL=/dev/[YOUR SERIAL PORT DEVICE]
```
Setting will be applied automatically from next shell starting or booting up.

#### Apply setting immediately

In your shell:

(bash)
```bash
source ~/.bashrc
```

(zsh)
```zsh
source ~/.zshrc
```

### Check the current setting
```sh
echo $TTY_WHILL  # -> Should be /dev/[YOUR SERIAL PORT DEVICE]
```

## Launch
```sh
roslaunch ros_whill modelc.launch
```

### Set serial port as an argument of the launch file
```sh
roslaunch ros_whill modelc.launch serialport:=/dev/[YOUR SERIAL PORT DEVICE]
```


### In the case of opening serial port failed

Edit
```
/lib/udev/rules.d/50-udev-default.rules
```

And add:
```
KERNEL=="ttyUSB[0-9]*", MODE="0666"
```

## Change Speed Profile
```sh
rosservice call /set_speed_profile {4, 15, 16, 64, 10, 16, 56, 10, 56, 72}
```
