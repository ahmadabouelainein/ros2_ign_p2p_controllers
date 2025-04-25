# Forklift Control with ROS 2 and Ignition Gazebo

This repository provides a complete, containerized solution for simulating and controlling a forklift model in Ignition Gazebo Fortress using ROS 2. The project demonstrates:

- A **Gazebo-compatible forklift URDF**, with sensor and actuator plugins.
- A **custom point-to-point (P2P) controller**.
- A **ROS 2 Action Server** (`/p2p_control`) exposing a `NavigateToPose` interface.
- **Unit and integration tests** for reliability.
- A **Dockerized environment** for reproducible builds and deployments.

---

## Motivation

In industrial robotics, rapid prototyping and validation in simulation accelerate development cycles and reduce hardware risk. Existing navigation stacks such as Nav2 can be heavyweight for simple point-to-point tasks. This project aims to deliver a lightweight, ROS 2-native controller optimized for:

- Minimal dependencies and easy customization
- Fast feedback loops and deterministic behavior
- Seamless integration with `ros2_control` and Ignition Gazebo Fortress

---

## System Architecture

Below is an overview of the components and data flow:

```text
+------------------------------------------------+
|                Docker Container                |
|                                                |
|  +---------------+      +-------------------+   |
|  | ROS 2 Action  |      | ign_ros2_control  |   |
|  | Server (/p2p)  |<---->| Bridge Plugin     |   |
|  +-------+-------+      +---------+---------+   |
|          |                        |             |
|          v                        v             |
|  +---------------+      +-------------------+   |
|  |  P2P Ctrl     |      | ros2_control      |   |
|  | Plugin (C++)  |----->| Controller Manager|   |
|  +---------------+      +--------+----------+   |
|          |                        |             |
|          v                        v             |
|  +-------------------------------------------+  |
|  | Ignition Gazebo Fortress Simulator       |  |
|  | (Forklift URDF, Sensors, Actuators)      |  |
|  +-------------------------------------------+  |
+------------------------------------------------+
```

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
- **ROS2 Humble**: Make sure you have ROS2 installed on your system.
- **Gazebo**: Install the appropriate version of Gazebo.
- **Docker**: For containerizing the setup.

### Instructions

1. Clone the repository:
   
   ```
   git clone https://github.com/ahmadabouelainein/ros2_ign_diffdrive/
   ```
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
    
## License

Copyright © 2023 ahmadabouelainein \
This project is available under the terms of [the MIT License](LICENSE).
   



   


