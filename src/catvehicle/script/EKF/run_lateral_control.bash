#! /bin/bash
gnome-terminal -- bash -c "killall gzserver" &
sleep 1 &&
echo ' Gazeobo starts running ... '

gnome-terminal -- bash -c "roslaunch carbot_gazebo carbot_world.launch" &
sleep 25 &&
echo ' Cat vehicle model is loading ... '

gnome-terminal -- bash -c "source /home/piwi/catkin_ws/devel/setup.bash && roslaunch catvehicle catvehicle_spawn.launch robot:=catvehicle X:=2 Y:=1.5 \yaw:=0" &
sleep 10 &&
echo ' data process node starts running ... '

gnome-terminal -- bash -c "rosrun catvehicle ssae_data_process.py" &
sleep 5 &&
echo ' sideslip angle estimation node starts running ... '



gnome-terminal -- bash -c "rosrun catvehicle SSAE_beta.py" &
sleep 5 &&
echo ' image process & vehicle control node starts running ... '


gnome-terminal -- bash -c "rqt_plot /SSAE/beta/data /SSAE/deltav/data" &
sleep 5 &&


gnome-terminal -- bash -c "rosrun catvehicle lateral_vehicle_control.py" &
sleep 5 &&


echo ' '
echo ' finish!'
echo ' '
