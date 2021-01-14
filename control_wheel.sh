#!/bin/sh

# 轮子底盘一键启动
ls -l /dev/input/js*

echo "Hi, man! Please input password for hardware authority: "
sudo chmod a+rw /dev/input/js0

sleep 3

echo "rosrun joy joy_node"
rosrun joy joy_node  &

sleep 3

echo "rosrun joy to twist command node"
rosrun xbox360_controller_interface joy_wheel_control_node  &

sleep 3

ls -l /dev/ttyUSB*
echo "Hi, man! Please input password for hardware authority: "
sudo chmod 777 /dev/ttyUSB0
rosrun ackerman_control serial_contact &

wait 

exit 0
