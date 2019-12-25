# ros_whill
ros_whill is a ROS package for [WHILL Model CR](https://whill.jp/model-cr).<br>
We also have [a FAQ and developers community website](https://whill.zendesk.com/hc/ja) for current and potential Model CR users.<br>
For general questions and requests, please visit https://whill.zendesk.com/hc/ja .

<img src="https://user-images.githubusercontent.com/2618822/45492944-89421c00-b7a8-11e8-9c92-22aa3f28f6e4.png" width=30%>

## Requirements
- ROS Melodic

## ROS API

### Subscribed Topics

#### ~controller/joy [(sensor_msgs/Joy)](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
- Virtual WHILL joystick input. You can controll WHILL via this topic.

#### ~controller/cmd_vel [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
- cmc_vel input. You can controll WHILL via this topic.
- This command is only available Model CR firmware updatedd after 2019.12. If you want to use this cmd_vel, please update firmware of Model CR by contact to sales of WHILL.

### Published Topics

#### ~states/joy [(sensor_msgs/Joy)](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
- Joystick status

#### ~states/jointState [(sensor_msgs/JointState)](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
- Wheel rotate position(rad) and rotation velocity(rad/s)

#### ~states/imu [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- IMU measured data.

#### ~states/batteryState [(sensor_msgs/BatteryState)](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
- Battery information


### Services

#### ~odom/clear [std_srvs/Empty]
Clear Odometry

#### ~power [std_srvs/SetBool]
True to send power on command, false to power off.

#### ~speedProfile/set [ros_whill/SetSpeedProfile]
You can set WHILL speed profile for `~controller/joy` topic.
```
ros_whill/SpeedPack forward
  float32 speed  # m/s
  float32 acc    # m/ss
  float32 dec    # m/ss
ros_whill/SpeedPack backward
  float32 speed  # m/s
  float32 acc    # m/ss
  float32 dec    # m/ss
ros_whill/SpeedPack turn
  float32 speed  # rad/s
  float32 acc    # rad/ss
  float32 dec    # rad/ss
---
bool success
string status_message

```

### Parameters

### ~init_speed/*
See: https://github.com/WHILL/ros_whill/blob/melodic-devel/params/initial_speedprofile.yaml

### ~keep_connected (Bool, default:false)
Set true to try to keep connected by re-opening port and sending power-on command. Though the WHILL automticarry wakes up even you turn off manualy or by power-off command.

### ~publish_tf (Bool, defualt: true)
False to stop publishing `odom` to `base_link` tf. If other node publishs, set value to false.

### ~serialport (String, default:/dev/ttyUSB0)


## SerialPort Setting for ros_whill.launch
The `ros_whill.launch` is using environmental variable `TTY_WHILL` for specify which serial port to be used.

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

### In the case of opening serial port failed

Edit
```
/lib/udev/rules.d/50-udev-default.rules
```

And add:
```
KERNEL=="ttyUSB[0-9]*", MODE="0666"
```


## Launch with Model
```sh
$ roslaunch ros_whill ros_whill.launch
```

### Set serial port as an argument of the launch file
```sh
roslaunch ros_whill ros_whill.launch serialport:=/dev/[YOUR SERIAL PORT DEVICE]
```
