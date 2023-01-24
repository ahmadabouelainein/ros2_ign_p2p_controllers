# DiffDrive ROS2 Foxy & Gazebo Ignition

A demo of how to integrate ROS2 with gazebo ignition in python.

## Requirements
To start using this demo  make sure you have [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) and [Gazebo Ignition Citadel](https://gazebosim.org/docs/citadel/ros2_integration).

## Environment Setup
Source the ROS2 envirnoment:
```
source /opt/ros/foxy/setup.bash
```
Clone the repository.
Go to the workspace: 
```
~/ros2_ign_diffdrive
```
In that directory build the package:
```
colcon build
```
In the same directory source the workspace by running in a terminal:
```
source install/setup.bash
```
Note: make sure you source the ros environment and your workspace everytime you open a terminal.

## Running
To launch the robot in the gazebo ignition simulator run:
```
ros2 launch mini_task spawn_robot_ign.launch.py
```
In another terminal launch the command publisher node:
```
ros2 launch mini_task twist_command_publisher.launch.py
```
Click the play button in the window simulator window to start the simulation.

Alternatively you launch everything using one file:
```
ros2 launch mini_task simulator.launch.py
```

## Features
 - To use a different robot model simply using the launch arguement ```ign_args:='path/your_robot_name.sdf'``` when launching the robot
 - The control input can be modified simply by modifying the parameter values in the ```src/mini_task/config/command.yaml```

## License

Copyright © 2023 ahmadabouelainein \
This project is available under the terms of [the MIT License](LICENSE).
