## Setup:

- Just like the labs, make a new directory in /ros_workspaces. 
- Init a catkin workspace in a new /src in that directory. 
- Build workspace. 
- Clone this repo into /src directory. 
- Rebuild workspace. 
- Clone stdr simulator into /src directory. 
- Install it from package root directory. 
- Rebuild workspace (this might take awhile).


### Copypasta:
```
mkdir -p ~/ros_workspaces/beeboop/src
cd ~/ros_workspaces/beeboop/src
catkin_init_workspace

cd ~/ros_workspaces/beeboop
catkin_make

cd ~/ros_workspaces/beeboop/src
git clone https://github.com/FreezingClouds/decentralized_search.git
cd ~/ros_workspaces/beeboop
catkin_make

cd ~/ros_workspaces/beeboop/src
git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git
cd ~/ros_workspaces/beeboop
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin_make

source devel/setup.bash
```

## Launching the simulation:

- Source the workspace setup.bash file.
- Roslaunch the launch file (currently named control_task_env) from this package.

### Copypasta:
```
cd ~/ros_workspaces/beeboop
source devel/setup.bash
roslaunch dencentralized_search control_task_env.launch
```

## Running the demo controller script:

- Ensure all scripts in this package's /src directory are executable.
- Source the workspace setup.bash file.
- Rosrun unicycle_control.py from this package with arguments 'robot0' and 'target'

This will make robot0 attempt to naively travel to the 'target' TF frame published to the /tf topic.

### Copypasta:
```
cd ~/ros_workspaces/beeboop/src/decentralized_search/src
chmod +x *.py
cd ~/ros_workspaces/beeboop
source devel/setup.bash
rosrun decentralized_search unicycle_control.py robot0 target
```
