#!/bin/bash

if [ -z "$GAZEBO_MODEL_PATH" ]; then
  bash -c 'echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"`pwd`/../assets/models >> ~/.bashrc'
else
  bash -c 'sed "s,GAZEBO_MODEL_PATH=[^;]*,'GAZEBO_MODEL_PATH=`pwd`/../assets/models'," -i ~/.bashrc'
fi

# add bebop launch environment variable
if [ -z "$GYM_GAZEBO_BEBOP_RELAY" ]; then
  bash -c 'echo "export GYM_GAZEBO_BEBOP_RELAY="`pwd`/../assets/worlds/relay.world >> ~/.bashrc'
else
  bash -c 'sed "s,GYM_GAZEBO_BEBOP_RELAY=[^;]*,'GYM_GAZEBO_BEBOP_RELAY=`pwd`/../assets/worlds/relay.world'," -i ~/.bashrc'
fi
 
exec bash # reload bash