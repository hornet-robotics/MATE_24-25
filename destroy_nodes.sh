#!/bin/bash

sshpass -p 'mate123' ssh mate-user@192.168.0.10 '
pkill -f vector_drive_subscriber
'
pkill -f joystick_publisher


echo "nodes destoryed"