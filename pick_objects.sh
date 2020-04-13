#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 8
# xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" &
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 4
xterm -e "roslaunch pick_objects pick_objects_test.launch "
# &
# xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"
