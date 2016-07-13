#! /bin/bash                                                                                                                                       

source /opt/ros/indigo/setup.bash
source devel/setup.bash

gnome-terminal \
--tab --title "pose_adapter" --command "bash -c 'rosrun pose_adapter pose_adapter_node;'" \
--tab --title "pendulum_ros" --command "bash -c 'rosrun pendulum_ros pendulum_ros_ctrl;'" \
--tab --title "pendulum_cfg" --command "bash -c 'rosrun rqt_reconfigure rqt_reconfigure;'"