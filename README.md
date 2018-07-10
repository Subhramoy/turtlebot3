# Turtlebot3

Turtlebot3 on Gazebo 9

## How to test

1. Clone the `master` branch
2. `$ roslaunch turtlebot3_gazebo turtlebot3_world.launch`
3. `$ roslaunch turtlebot3_slam turtlebot3_slam.launch`
4. `$ rviz -d ``rospack find turtlebot3_slam``//turtlebot3_slam.rviz`
5. `$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

