# Note
The gz_ros_control directory is a copy of this [repo](https://github.com/ros-controls/gz_ros2_control/tree/humble) 

# Forklift Control with ROS2 and Gazebo

This repository provides a solution for simulating and controlling a forklift model in Gazebo using ROS2. The project includes several key features that enhance the forklift’s simulation and control, ensuring a seamless integration into the Gazebo environment.

## Features

### 1. **Gazebo-Compatible Forklift URDF**
   - The forklift URDF model has been modified to ensure full compatibility with Gazebo Fortress.
   - Necessary ros2 plugins are integrated to handle sensors, actuators, and interactions within the simulation environment.


### 2. **Differential Drive Offset Point-to-Point (P2P) Controller**
   - A differential drive controller is implemented to move the forklift from one point to another using a point-to-point (P2P) control strategy.
   - The controller accounts for differential drive system offsets, ensuring precise control over the forklift’s movement.

### 3. **Unit and Integration Testing**
   - Unit tests are implemented for control algorithms and system behavior to ensure reliability.
   - Integration tests validate the interaction between the forklift model, the controller, and the Gazebo simulation, confirming that all components work seamlessly together.

### 4. **Dockerized Setup**
   - The entire setup is containerized using Docker, providing a consistent and portable environment for easy deployment.
   - A `Dockerfile` is included for building the container and detailed instructions are provided for running the system within a Docker container.

## Setup and Installation

### Prerequisites

- **ROS2 Humble**: Make sure you have ROS2 installed on your system.
- **Gazebo**: Install the appropriate version of Gazebo.
- **Docker**: For containerizing the setup.

### Instructions

1. Clone the repository:
   
   ```
   git clone https://github.com/ahmadabouelainein/ros2_ign_diffdrive/
   ```
2. To build the Docker image:
    ```
    docker build -t p2p_forklift:nav  .
    ```
3. To run the system in Docker:
    ```
    docker run -it --rm --name p2p_container -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix p2p_forklift:nav 
    ```
    This would automatically run the launch file which opens gazebo with the forklift loaded in it. 
4. In another terminal run:
    ```
    docker exec -it p2p_container bash
    ```
    Once the terminal is attached to the container run:
    ```
    source install/setup.bash && ros2 action send_goal /p2p_control nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 20.2, y: 10.1, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}
    ``` 
    
# DiffDrive ROS2 & Gazebo Fortress

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
   


