# TurtleBot Simulation which Exhibits a Simple Walker Algorithm
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

---

## Overview

A ROS package consisting of:
- TurtleBot simulation which exhibits a simple walker algorithm much like a Roomba robot vacuum cleaner

## License
```
BSD 3-Clause License

Copyright (c) 2018, Rohith Jayarajan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Dependencies

### ROS

- [ROS][reference-id-for-ROS] The Robot Operating System (ROS) is a flexible 
framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify 
the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
This package is developed and tested in ROS Kinetic on Ubuntu 16.04 LTS and needs ROS Kinetic Kame installed for use. 
The entire installation instructions for ROS Kinetic Kame and its dependencies can be found [here][reference-id-for-ROS Kinetic].

- [Gazebo][reference-id-for-Gazebo]: Gazebo is included by default when ROS is installed. It is a set of ROS packages that provide the necessary interfaces to simulate a robot in the Gazebo 3D rigid body simulator for robots. Installation instructions can be found [here][reference-id-for-GazeboInstall]

[reference-id-for-ROS Kinetic]: http://wiki.ros.org/kinetic
[reference-id-for-ROS]: http://www.ros.org/install/
[reference-id-for-Gazebo]: http://gazebosim.org/
[reference-id-for-GazeboInstall]: http://gazebosim.org/tutorials?tut=ros_installing


### catkin

Catkin is included by default when ROS is installed. Catkin can also be installed from source or prebuilt packages. 
Most users will want to use the prebuilt packages, but installing it from source is also quite simple. Installation 
instructions can be found [here][reference-id-for-catkin]

[reference-id-for-catkin]: http://wiki.ros.org/catkin

### Package Dependencies
- roscpp
- rospy
- std_msgs
- sensor_msgs
- turtlebot packages: These can be installed in ROS Kinetic running on Ubuntu 16.04 by using the below command in the terminal

```
sudo apt-get install ros-kinetic-turtlebot-*
```

## Build Instructions

### Creating catkin workspace:
Follow the below comamnds to create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
The above commands will create a workspace for your packages with CMakeLists.txt link in the src folder of catkin_ws. 
Source the setup.*sh file: 
```
source devel/setup.bash
```
### Building package inside catkin workspace: 
Follow the below comamnds and clone this package in the src folder of the catkin workspace 
```
cd ~/catkin_ws/src/
git clone https://github.com/rohithjayarajan/turtlebot_roomba_walker.git
```
Follow the below comamnds to build the package
```
cd ~/catkin_ws/
catkin_make
```

## Run Instructions

Follow the below commands in the terminal to run TurtleBot simulation which exhibits a simple walker algorithm and displays the simulation in Gazebo (rosbag recording can be enabled here. Default=False; duration of recording bag file can be modified here. Default=30):
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_roomba_walker turtlebot_roomba_walker.launch record:=<true_to_enable_bagfile_recording> record_time:=<duration_of_bag_recording>
```


## Logging

Once the node is running in the background using the roslaunch method, to visualize the logger messages in a GUI, in a new terminal follow the below commands

```
source devel/setup.bash
rosrun rqt_console rqt_console
```

Once the node is running in the background using the roslaunch method, to visualize logger_level GUI, in a new terminal follow the below commands

```
source devel/setup.bash
rosrun rqt_logger_level rqt_logger_level
```

## Recording bag Files with the Launch File

To record a rosbag containing all topics being published, follow the steps below in a new terminal:

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_roomba_walker turtlebot_roomba_walker.launch record:=true
```
BY default, the bag file will be recorded for 30 seconds. To change duration of recording time, refer to `Run Instructions` above. Press `Ctrl + C` on the terminal above to kill the nodes and Gazebo simulator.

Toggling the `record` parameter will enable or disable recording of a rosbag using the `roslaunch` command. Setting `record` to `true` will enable rosbag recording and setting `record` to `false` will disable rosbag recording. Default value of `record` parameter is `false` (rosbag is not recorded in default case)

## Inspecting the bag File

`rosbag info <your bagfile>` helps to inspect a recorded rosbag. In our case, the recorded bag file is record_all.bag. Open the terminal in the folder where the .bag file of interest is present and use the following command to inspect it:

```
cd ~/catkin_ws/src/turtlebot_roomba_walker/results
rosbag info record.bag
```

An output similar to the below is produced on following the above command

```
path:        record.bag
version:     2.0
duration:    29.9s
start:       Dec 31 1969 19:00:00.34 (0.34)
end:         Dec 31 1969 19:00:30.25 (30.25)
size:        11.8 MB
messages:    23215
compression: none [16/16 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            2991 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               2991 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              2991 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     2870 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     58 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     449 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     2873 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  116 msgs    : bond/Status                           (3 connections)
             /odom                                             2873 msgs    : nav_msgs/Odometry                    
             /rosout                                            515 msgs    : rosgraph_msgs/Log                     (10 connections)
             /rosout_agg                                        498 msgs    : rosgraph_msgs/Log                    
             /scan                                              275 msgs    : sensor_msgs/LaserScan                
             /tf                                               3705 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage
```


## Playing Back the bag File

Once the bag file of publisher topics has been generated and saved, use the below commands to play back the bag file with the turtlebot_roomba_walker node
Note for bag playback Gazebo should not be running

In a new terminal, follow the below command

```
roscore
```

In a new terminal, follow the below commands

```
cd ~/catkin_ws/src/turtlebot_roomba_walker/results
rosbag play record.bag
```


## Killing Processes

Kill the above three processes by pressing CTRL+C in the aforementioned terminals where roscore and rosrun have been run.
Another way to kill the nodes is by running the below command in a new terminal

```
rosnode kill [node_name]
```
