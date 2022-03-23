# roko_ros

## Running the code
Alright, to run the basic client-server-client example from this repository you will need to open three terminal windows.
In the first one - launch ROSCORE:
```
rocore
```
In the second window - run the server node:
```
rosrun roko_robot roko_node.py
```
In the third window you can check the topics created by this node:
```
rostopic list
```
And you can check if the sensors topic contains the correct sensor measurements data
```
rostopic echo /sensors_data
```
