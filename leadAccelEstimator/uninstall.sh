#!/bin/bash

echo "=========================="
echo "Removing App leadAccelEstimator"


LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable gps2vsl


# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
cd
if [ "x"`systemctl list-units | grep -c gps2vsl.service` = "x1" ]; then
    echo "Uninstalling gps2vsl.service"

    source /home/$LIBPANDA_USER/catkin_ws/devel/setup.bash
    rosrun robot_upstart uninstall gps2vsl
fi

systemctl daemon-reload # if needed
