#!/bin/bash
gnome-terminal -- bash -c "pkill -f 'gz sim'; exec bash" &
sleep 1
echo "Gazebo starts running ..."
gnome-terminal -- bash -c " source /opt/ros/kilted/setup.bash source /home/student/ROS2/install/setup.bash ros2 launch carbot_gazebo carbot_world.launch.py exec bash " &
sleep 25
echo "CAT vehicle model is loading ..."
gnome-terminal -- bash -c " source /opt/ros/kilted/setup.bash source /home/student/ROS2/install/setup.bash ros2 launch catvehicle catvehicle_spawn.launch.py \ robot:=catvehicle \ X:=2.0 \ Y:=1.5 \ yaw:=0.0 exec bash " &
sleep 10
echo "Data process node starts running ..."
gnome-terminal -- bash -c " source /opt/ros/kilted/setup.bash source /home/student/ROS2/install/setup.bash ros2 run catvehicle ssae_data_process.py exec bash " &
sleep 5
echo "Sideslip angle estimation node starts running ..."
gnome-terminal -- bash -c " source /opt/ros/kilted/setup.bash source /home/student/ROS2/install/setup.bash ros2 run catvehicle SSAE_beta.py exec bash " &
sleep 5
echo "Image process & vehicle control node starts running ..."
gnome-terminal -- bash -c " source /opt/ros/kilted/setup.bash source /home/student/ROS2/install/setup.bash rqt_plot /SSAE/beta/data /SSAE/deltav/data exec bash " &
sleep 5
gnome-terminal -- bash -c " source /opt/ros/kilted/setup.bash source /home/student/ROS2/install/setup.bash ros2 run catvehicle lateral_vehicle_control.py exec bash " &
sleep 5
echo ""
echo "finish!"
echo ""
```
