How to setup:

- Just like the labs, make a new directory in /ros_workspaces. 
- Init a catkin workspace in a new /src in that directory. 
- Build workspace. 
- Clone this repo into /src directory. 
- Rebuild workspace. 
- Clone stdr simulator into /src directory. 
- Install it from package root directory. 
- Rebuild workspace. 


Copypasta:
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
