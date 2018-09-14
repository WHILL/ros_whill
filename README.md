# ros_whill
<img src="https://user-images.githubusercontent.com/20053970/45531217-981ce300-b829-11e8-8253-a1961eb58897.png" width=30%>

## Overview
This ROS package provides a ROS node that communicates with WHILL Model CR by RS232C data line.


## Supported Hardware
WHILL Model CR (Normal Model C does not support serial communication.)

## Getting Started
There are launch file for this node:
```
roslaunch ros_whill whill_modelc.launch
```

### Published Topics:
/whill_modelc_msg

### Parameters:
port (string, default: "/dev/ttyUSB0") Serial port.
