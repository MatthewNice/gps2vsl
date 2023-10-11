#!/bin/bash

echo "=========================="
echo "Removing App middleway"


LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable middleway


# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
cd
if [ "x"`systemctl list-units | grep -c middlway.service` = "x1" ]; then
    echo "Uninstalling can.service"

    source /home/$LIBPANDA_USER/catkin_ws/devel/setup.bash
    rosrun robot_upstart uninstall middleway
fi

systemctl daemon-reload # if needed
