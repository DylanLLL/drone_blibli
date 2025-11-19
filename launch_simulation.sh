#!/bin/bash

# Set GZ_SIM_RESOURCE_PATH if not already set (for custom models)
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/home/gdnuser/PX4-Autopilot/Tools/simulation/gz/models

#aruco, warehouse
sim_choice="warehouse"

# Launch QGroundControl in a new terminal
gnome-terminal --title="QGroundControl" -- bash -c "cd ~/Downloads && ./QGroundControl.AppImage; exec bash"

# Small delay to let QGC start
sleep 5

if [ "$sim_choice" == "warehouse" ]
then
# Launch PX4 SITL with custom world, safe spawn pose, no GPS, and vision model
gnome-terminal --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_MODEL_POSE="0,0.45,0,0,0,1.57" PX4_GZ_WORLD=warehouse make px4_sitl gz_x500_mono_cam_down; exec bash"

elif [ "$sim_choice" == "aruco" ]
then
gnome-terminal --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_MODEL_POSE="0,0.45,0,0,0,1.57" PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down; exec bash"
fi


# Longer delay to let SITL fully initialize and EKF2 start
sleep 15

# Launch micro ros agent in a new terminal
gnome-terminal --title="micro_ros_agent" -- bash -c "cd ~/microros_ws && source install/local_setup.sh && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash"

# PX4_GZ_MODEL_POSE="0,0,0,0,1.57"

# Launch Micro XRCE Agent in a new terminal
#gnome-terminal --title="micro_ros_agent" -- bash -c "echo 'Porschespyder918!' | sudo -S /usr/local/bin/MicroXRCEAgent udp4 -p 8888; exec bash"