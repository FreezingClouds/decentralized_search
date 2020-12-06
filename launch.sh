#!/bin/bash
cd ~/ros_workspaces/beeboop
source devel/setup.bash
roslaunch decentralized_search control_task_env.launch &
sleep .1
roslaunch decentralized_search pursuer_1.launch &
sleep .1
roslaunch decentralized_search pursuer_2.launch &
sleep .1
roslaunch decentralized_search pursuer_3.launch &
wait