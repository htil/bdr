#!/usr/bin/env bash

ROSCORE="bash -c 'source ~/.bashrc; echo roslaunch bebop_driver bebop_node.launch; roslaunch bebop_driver bebop_node.launch; bash'"
CONTROLLER="bash -c 'source ~/.bashrc; echo rosrun bebop_brain_drone_race sensor.py; rosrun bebop_brain_drone_race sensor.py; bash'"
SENSOR="bash -c 'source ~/.bashrc; echo rosrun bebop_brain_drone_race controller.py; rosrun bebop_brain_drone_race controller.py; bash'"
RQT_GUI="bash -c 'source ~/.bashrc; echo rosrun rqt_image_view rqt_image_view; rosrun rqt_image_view rqt_image_view; bash'"
CAMERA="bash -c 'source ~/.bashrc; echo adjusting the camera; rostopic pub /bebop/camera_control geometry_msgs/Twist \"{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: -90, z: 0}}\"; bash'"
EMERGENCY="rostopic pub --once /bebop/reset std_msgs/Empty"

gnome-terminal --tab -e "$ROSCORE"
sleep 5
gnome-terminal --tab -e "$SENSOR" --tab -e "$CONTROLLER" --tab -e "$RQT_GUI" --tab -e "$CAMERA"



