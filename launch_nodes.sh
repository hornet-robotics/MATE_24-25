#!/bin/bash

# note: run this from laptop not the robot's pi

# scrip to launch full ROS workspace
# - nodes will be launched
# - ssh session will be made independent
# - arduino presence will be checked

# launch pi nodes

# make pi nodes independent of ssh terminal state
# ssh into pi at its static ip on the tp-link router then run pi setup commands and exit ssh session
sshpass -p 'mate123' ssh mate-user@192.168.0.10 '
# check if arduino is connected, if so elevate permissions 
if (ls /dev/ | grep -q ttyACM0); then
    echo "mate123" | sudo -S chmod a+rw /dev/ttyACM0
    echo "Arduino found"
    echo "launched Pi Nodes"
else
    echo "!!!ERROR: Connect Arduino and run script again to launch ROS Nodes."
    exit 1
fi

source /opt/ros/jazzy/setup.bash;
source ~/Software/MATE_24-25/mate_workspace/install/setup.bash;
# run in background and ignore standard output and error output
nohup ros2 run vector_drive vector_drive_subscriber > /dev/null 2>&1 &
'
< /dev/null # don't expect input (lets ssh session close)

# launch laptop nodes
if (lsusb | grep -q "Xbox360 Controller" > /dev/null); then
    nohup ros2 run joystick_module joystick_publisher > /dev/null 2>&1 & # redirect standeard output and standard error output to null
    echo "launched TopSide Nodes"
else
    echo "!!!ERROR: Connect Xbox Controller and run script again to launch ROS Nodes."
    exit 1
fi
