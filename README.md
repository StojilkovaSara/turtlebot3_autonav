# TurtleBot3 Autonomous Navigation

This project demonstrates navigation of a **TurtleBot3 Waffle** robot inside a **Gazebo environment** using **ROS Melodic**. The robot can either be launched in the **turtlebot3_world** where it autonomously navigates to predefined goal points,  or in any world  where it explores and maps the space with **Frontier-Based Exploration**.

---

## Project Structure

turtlebot3_autonav

|---include/

|---launch/

|---scripts/

|---|---paths/
    
|---src/

|---CMakeLists.txt/

|---package.xml/

Currently, all the logic is in **launch** and **scripts**, while 'src/' is empty since no C++ nodes are implemented yet.

---

## Requirements

- **Ubunty 18.04** (recommended)
- **ROS 1 MELODIC** installed
    - Install instructions: [ROS MELODIC INSTALATION GUIDE](http://wiki.ros.org/melodic/Installation/Ubuntu)

---

# Installation

- Create a ROS workspace (if not already done):
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
- Clone or copy this package into your workspace
```bash
cd ~/catkin_ws/src
git clone https://github/StojilkovaSara/turtlebot3_autonav.git turtlebot3_autonav
```
- Install all system dependencies declared in package.xml using rosdep:
```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
- Source the workspace
```bash
source devel/setup.bash
```
- or to make it permanent
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# Usage

## Navigation to One Point Simulation

This simulation sends the robot to a certain point on the map. Follow the next instructions to send the robot to a point.

- Make sure the python script is executable
```bash
chmod +x scripts/send_goal.py
```
- Launch **send_goal** with the wanted coordinates. You can also leave out the arguments, and in that case the robot will be sent to a default point 0.0, 2.0, 0.0.
```bash
roslaunch turtlebot3_autonav send_goal.launch x:=0.0 y:=2.0 yaw:=1.57
```
- Once the robot reaches the goal, you can send it another goal by just running the send_goal python script again
```bash
rosrun turtlebot3_autonav send_goal.py 2.0 1.5
```
## Multi Point Navigation Simulation

This simulation can be run with any path defined in **scripts/paths/**. The following instructions will send the robot on a path going around all nine obsticles, but feel free to change the argument of the launch command with any other path.yaml files in the directory.

- Make sure the script is executable
```bash
chmod +x scripts/multi_point_nav.py
```
- Launch **tb3_autonav**
```bash
roslaunch turtlebot3_autonav tb3_autonav.launch path_file:=all_around.yaml
```
- Once the robot has reached the final destination, you can send it on another path by just running the multi_point_nav python script again
```bash
rosrun turtlebot3_autonav multi_point_nav.py (path to path_file)
```

## Exploring Simulation

This simulation can be run with any gazebo world. The following instructions are for simulating the robot exploring the **turtlebot3_house** world, but feel free to change the argument with any other world you want to try out.

- Make sure the script is executable. We need this script to make sure that explore_lite is launched after move_base has finished launching.
```bash
chmod +x scripts/launch_explore.py
```
- Launch **explore_map.launch**
```bash
roslaunch turtlebot3_autonav explore_map.launch world_launch:=turtlebot3_house.launch
```

---

# Author

- Sara Stojilkova
- [GitHub Profile](https://github.com/StojilkovaSara)

