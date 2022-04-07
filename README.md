# roko_ros
## Project structure
This project contains two ros packages: **roko_robot** and **robot_software**.
### roko_robot
This package imitates the real robot behavior (hardware simulation).
It uses math to compute the robot motion and to process incoming control commands.
It also applies noisy measurement models to simulate robot's sensors.
### robot_software
This package contains high-level software of the robot - its brains.
This package should include the navigation, control and general data processing algorithms.
It also accounts for all the decision making.

Right now this package contains the following nodes:
* control_node

## Setup
Place the contents of this repo to the */src folder* of your catkin workspace.
Navigate to the root of your catkin workspace and run
```
catkin_make
```
to build the ros packages. If you see any compilation errors - it is likely that the cause is custom message dependacies.
It should be easily fixed by running
```
catkin_make
```
one more time. If you have any futher errors - please sumbit an issue to this repo.

## Launching all at once
You can easily launch the whole package with a single command
```
roslaunch robot_software navigation.launch
```
This will launch robot simulation node **roko_node**, control algorithm **control_node** and
**navigation_node** as well as predefined Rviz window for visualization.

## Running the nodes one by one for easier debug
First thing you need to do - is to launch ROSCORE
```
roscore
```
Open the second terminal window and launch **roko_node** to start simulating robot behaviour:
```
rosrun roko_robot roko_node.py
```
In the third terminal window you can run the **control_node**:
```
rosrun robot_software control_node
```
In the fourth terminal run the **navigation_node**:
```
rosrun robot_software navigation_node
```
You can check the sensors data by monitoring the topic:
```
rostopic echo /roko/sensors_data
```
Also you can send control commands manually without using the control node:
```
rostopic pub -1 /control_data roko_robot/control 0 2 1
```
This will tell the robot that the desired rotation speed of the left wheel equals to 2 rad/s and for the right wheel - 1 rad/s.
