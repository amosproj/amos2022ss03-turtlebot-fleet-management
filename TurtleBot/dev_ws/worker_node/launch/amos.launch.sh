#!/bin/bash

ros2 launch turtlebot2_ros2 bringup.launch.py > /dev/null 2>&1 &
ros2 run sick_lidar_localization sick_lidar_localization src/sick_lidar_localization/launch/sick_lidar_localization.launch > /dev/null 2>&1 &
ros2 run worker_node worker
