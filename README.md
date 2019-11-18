# Robot Vaccum Cleaner
[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/RajPShinde/robot_vaccum_cleaner/blob/master/LICENSE)

## Authors

**Raj Prakash Shinde** - [GitHub](https://github.com/RajPShinde)
<br>I am a Masters in Robotics Engineering student at the University of Maryland, College Park. My primary area of interest are Legged Robotics and Automation. 

## Overview
This is an Obstacle avoidance robot vaccum cleaner. Here a turtlebot is simulated in gazebo, where the data from laser is used to measure distance and than avoid obstacles. A walker node is created, that subscribes to the sensor to get range and also publishes velocity and rotation when an obstacle is detected.

## Dependencies
1. Ubuntu 16.04
2. ROS Kinetic
3. Turtlebot stack
<br> Install by running following command
```

sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

## Build
Steps to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/RajPShinde/robot_vaccum_cleaner
cd ~/catkin_ws/
catkin_make
```
## Rosbag Link
Google drive- https://drive.google.com/file/d/1X70HMTfUhdhIQ-KywGHGyjXs5tqmQE0u/view?usp=sharing

## Run
1. Launch using launch file
(Open a new Terminal)
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch robot_vaccum_cleaner SimLauncher.launch
```
2. To Enable recording in a bag file, we can pass a record argument to the launch file
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch robot_vaccum_cleaner SimLauncher.launch record:-true
```
To avoid Recording, we can either pass false or dont pass any argument at all
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch robot_vaccum_cleaner SimLauncher.launch record:-false
```
OR
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch robot_vaccum_cleaner SimLauncher.launch
```
3. To inspect Rosbag file(Open a new terminal)
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/robot_vaccum_cleaner/results/
rosbag info recording.bag 
```

4. To play the Rosbag
Run roscore
```
roscore
```
Open a new terminal and play
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/robot_vaccum_cleaner/results/
rosbag play recording.bag 
```

4. To view Log levels using rqt console and rqt logger level
Open a new Terminal
```
rosrun rqt_console rqt_console
```
Open a new Terminal
```
rosrun rqt_logger_level rqt_logger_level
```

## Reference
* http://wiki.ros.org/