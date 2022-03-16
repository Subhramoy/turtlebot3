# Project Title: Mapless Collision Avoidance of Turtlebot3 Using DDPG and Prioritized Experience Replay with Noisy Laser Sensor in Robots
A preliminary version of this code was developed by the authors of [Voronoi-Based Multi-Robot Autonomous Exploration in Unknown Environments via Deep Reinforcement Learning](https://ieeexplore.ieee.org/abstract/document/9244647) published in IEEE Transactions on Vehicular Technology.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

### Prerequisites
What things you need to install the software

```
Ubuntu 16.04
ROS Kinetic
Tensorflow-gpu == 1.13.1 or 1.14.0
Keras == 2.3.1
```
### Installing

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

## Git repo for noisy laser sensor integration:

checkout branch <noisy_laser>

## Git repo for camera and noisy laser sensor integration:
checkout branch <camera_noisy_laser>

```
$ cd ~/catkin_ws/src/
$ git clone <...>
$ cd ~/catkin_ws && catkin_make
```
## Notes for training and testing

For training
```
please change train_indicator=1 under ddpg_network_turtlebot3_amcl_fd_replay_human.py
```
For testing
```
please change train_indicator=0 under ddpg_network_turtlebot3_amcl_fd_replay_human.py
```

### Start gazebo world (Change your world file location based on your setting)

Terminal #1
```
$ export TURTLEBOT3_MODEL=waffle_pi 
$ roslaunch turtlebot_ddpg turtlebot3_empty_world.launch world_file:='/home/catkin_ws/src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/worlds/turtlebot3_modified_maze.world' 
```
### Start the RL for robot navigation (training/testing)
Terminal #2
```
$ cd ~/catkin_ws/src/.../turtlebot_ddpg/scripts/fd_replay/play_human_data
$ rosrun turtlebot_ddpg ddpg_network_turtlebot3_amcl_fd_replay_human.py
