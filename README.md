# WIT-IMU for ros2
    
This is a ROS 2 implementation of WIT IMU driver package.  
Support Devices:
- JY901
- WT901C (tested)

# Dependency

Serial Communication Library from https://github.com/RoverRobotics-forks/serial-ros2 

To install this library  

```
$ git clone https://github.com/RoverRobotics-forks/serial-ros2.git
$ cd serial-ros2
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install
```

# Installation

Clone this project into your ros2 workspace and build
```
$ cd to ros2_ws/src
$ git clone https://github.com/kyuhyong/WIT-IMU.git -b foxy
$ cd ..
$ colcon build --symlink-install
```
This will install wit-imu-driver package.  
Please don't forget to source the **install/setup.bash** file for the first time build.

# Device setup

It is required to properly setup the imu device using WIT's proprietary software from below link.  
https://www.wit-motion.com/digital-inclinometer/witmotion-wt901c-ttl-9-axis-imu-sensor.html

You will need to check these parameters
- Baudrate: default is 9600
- Output Hz: default is 10
- Magnetometer output
- Quaternion output: need to be set for TF 

# Launch node

wt901c.launch.py is provided to run wit-imu-driver_node with wt901c.yaml file in /config.  

```
$ ros2 launch wit-imu-driver wt901c.launch.py
```
wt901c.yaml file includes parameters you can modify per the device setup.  


# Run node

To just run the node, simply enter

```
$ ros2 run wit-imu-driver wit-imu-driver_node
```

# Troubleshoot

If you see messages like "Unable to open port " 
- If you find the device in /dev/ttyUSB0, You may try
```
$ sudo adduser {USER_NAME} dialout
```
- If you see multiple instances like /dev/ttyUSB1 or /dev/ttyUSB2,  
 Try different port name in /config/wt901c.yaml file
