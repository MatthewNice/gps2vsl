#!/bin/bash
# Author: Matt Bunting

LAUNCH_FILE=vehicle_control_integrated_cbf_vsl.launch

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

source ~/.bashrc

pushd ~/catkin_ws
source devel/setup.sh

rosrun robot_upstart install can_to_ros/launch/${LAUNCH_FILE} --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can
popd

echo "----------------------------"

