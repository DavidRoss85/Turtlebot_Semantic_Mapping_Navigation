#!/bin/bash
echo "Starting semantic map sequence..."
echo "Updating permissions..."
chmod +x ./launch_robo_keyboard.sh ./launch_nav_bringup.sh ./launch_nav_setup_node.sh ./launch_nav2_navigator.sh ./launch_nav2_slam_toolbox.sh ./launch_robosync_node.sh ./launch_detection_node.sh ./launch_location_node.sh ./launch_map_node.sh ./launch_map_viewer.sh ./launch_temp_viewer.sh
sleep 3

echo "Launching Nav2 setup node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_nav_setup_node.sh; exec bash"
sleep 10
echo "Launching slam toolbox in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_nav2_slam_toolbox.sh; exec bash"
echo "Waiting for SLAM to stabilize..."
sleep 10
echo "Launching navigation node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_nav2_navigator.sh; exec bash"
sleep 20 
# echo "Launching Nav2 stack with SLAM..."
# gnome-terminal --working-directory="$PWD" -- bash -c "./launch_nav_bringup.sh; exec bash"
# sleep 15

echo "Launching Robosync node in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_robosync_node.sh; exec bash"
echo "Launching Detection Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_detection_node.sh; exec bash"
echo "Launching Location node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_location_node.sh; exec bash"
echo "Launching Mapping node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_node.sh; exec bash"
echo "Launching viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_viewer.sh; exec bash"
echo "Launching temp viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_temp_viewer.sh; exec bash"
echo "Launching keyboard teleop in new terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_robo_keyboard.sh; exec bash"
echo "Startup complete."

