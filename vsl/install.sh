#!/bin/bash

echo "=========================="
echo "Installing App vsl"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for VSL..."

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

source /home/$LIBPANDA_USER/.bashrc

apt install -y python3-pandas
pip3 install cantools shapely

if [ -d /home/$LIBPANDA_USER/strym ]; then
    pushd /home/$LIBPANDA_USER/strym
    git pull
else
    pushd /home/$LIBPANDA_USER/
    git clone https://github.com/jmscslgroup/strym
fi
popd

runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/vsl/installRosPackagesForVsl.sh

echo "Installing VSL demo..."
runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/vsl/installVslController.sh
