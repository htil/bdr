#!/usr/bin/env bash

ROSCORE="bash -c 'echo roslaunch bebop_driver bebop_node.launch ; roslaunch bebop_driver bebop_node.launch; bash'"
CONTROLLER="bash -c 'echo rosrun bebop_brain_drone_race sensor.py; rosrun bebop_brain_drone_race sensor.py; bash'"
SENSOR="bash -c 'echo rosrun bebop_brain_drone_race controller.py; rosrun bebop_brain_drone_race controller.py; bash'"
RQT_GUI="bash -c 'echo rosrun rqt_gui rqt_gui; rosrun rqt_qui rqt_gui; bash'"
CAMERA="bash -c 'echo rostopic pub --once /bebop/reset std_msgs/Empty; rostopic pub --once /bebop/reset std_msgs/Empty;'"

gnome-terminal --tab -e "$ROSCORE"
gnome-terminal --tab -e "$CONTROLLER" --tab -e "$SENSOR" --tab -e "$RQT_GUI" --tab -e "$CAMERA"



