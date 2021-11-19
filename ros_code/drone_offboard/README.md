# Drone Offboard Package

A package for offboard drone control.

## Prerequisit
Install ROS Melodic and [PX4 SITL](https://docs.px4.io/master/en/simulation/gazebo.html).

## Installation
First, git clone this repository and move the ros package to your `catkin_ws`.
```bash
sudo apt update
sudo apt install git
cd ~/
git clone https://github.com/swl017/EE478.git
mv ros_code/drone_offboard/ ~/catkin_ws/src
```
Now your code is ready. Install listed dependencies at once with the following command and build using `catkin_make`.
```
cd ~/catkin_ws
rosdep install --from-paths src/ --ignore-src -r -y
catkin_make
```

## Running with the Simulation
Go to the PX4 code directory (probably named `PX4-Autopilot`).\
Run the following command to start PX4 Gazebo simulation
```bash
roslaunch px4 posix_sitl.launch
```
> Check if the following paths and variables are set to your terminal or in your `~/.bashrc`.
> ```bash
> gedit ~/.bashrc # Open bashrc and add the following.
> PX4_DIRECTORY="<absolute path to your PX4-AUTOPILOT directory>"
> source $PX4_DIRECTORY/Tools/setup_gazebo.bash $PX4_DIRECTORY $PX4_DIRECTORY/build/px4_sitl_default
> export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_DIRECTORY
> export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_DIRECTORY/Tools/sitl_gazebo
> ```

In another terminal, run the following command to connect to the drone in the simulation and retrieve topics.
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

You may use any node between `offboard_node_py.py` or `drone_offboard_node` by running
```bash
rosrun drone_offboard offboard_node_py.py
```
or
```bash
rosrun drone_offboard drone_offboard_node
```

> `offboard_node_py.py` is written in Python, and has more functionalities. Recommanded to start with this if you are not familiar with ROS or C++. \
`drone_offboard_node` is the example offboard code from the [PX4 official documentation](https://docs.px4.io/master/en/ros/mavros_offboard.html) wirtten in C++. It is written in the bare minimum, so interested students may upgrade the code by adding functions to fit their needs or turn the code into a class for efficient coding.

The drone should arm and takeoff.
