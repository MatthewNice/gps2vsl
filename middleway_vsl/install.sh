#!/bin/bash

echo "=========================="
echo "Installing App middleway"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for VSL..."

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)
LAUNCH_FILE=middleway_vsl.launch

source /home/$LIBPANDA_USER/.bashrc

if [ -d /home/$LIBPANDA_USER/strym ]; then
    pushd /home/$LIBPANDA_USER/strym
    git pull
else
    pushd /home/$LIBPANDA_USER/
    git clone https://github.com/jmscslgroup/strym
fi
popd

runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/vsl/installRosPackagesForMidVsl.sh

echo "Installing Middleway VSL demo..."
# runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/vsl/installMidVslController.sh
pushd /home/$LIBPANDA_USER/catkin_ws
runuser -l $LIBPANDA_USER -c 'source /opt/ros/noetic/setup.bash && catkin_make'
source devel/setup.sh
rosrun robot_upstart install ${LAUNCH_FILE} --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable middleway
popd
echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
