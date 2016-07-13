#! /bin/bash

source /opt/ros/indigo/setup.bash

set -x

gnome-terminal \
--tab --title "roscore" --command "bash -c 'roscore;'" \
--tab --title "mavros" --command "bash -c 'rosrun mavros mavros_node _fcu_url:=udp://localhost:14540@localhost:14557 _gcs_url:=udp://@localhost:14550;'" \
--tab --title "gazebo" --command "bash -c 'make posix_sitl_default gazebo;'"
 
