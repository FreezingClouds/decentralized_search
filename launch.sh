#!/bin/bash
cd ~/ros_workspaces/beeboop
source devel/setup.bash
roslaunch decentralized_search control_task_env.launch &
sleep 1
roslaunch --wait decentralized_search pursuer_2.launch &
sleep 1
roslaunch --wait decentralized_search pursuer_3.launch &
sleep 1
roslaunch --wait decentralized_search evader.launch &
sleep 1
rosrun rviz rviz
wait