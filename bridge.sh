#!/bin/bash

gnome-terminal --tab --title="ros1_nodes" -- bash -c "source ros.sh; roslaunch rt2_assignment1 sim2.launch"
gnome-terminal --tab --title="bridgeee" -- bash -c "sleep 15; source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2_nodes" -- bash -c "sleep 20; source ros2.sh; ros2 launch rt2_assignment1_2 ros2nodes_launch.py"
