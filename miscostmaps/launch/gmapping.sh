#!/bin/bash
xterm -e roscore &
sleep 3
xterm -e rosrun stage_ros stageros 'rospack find stage_ros'/world/willow-erratic.world
sleep 3
xterm -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py
sleep 1
xterm -e rosrun gmapping slam_gmappingscan:=base_scan
sleep 3
