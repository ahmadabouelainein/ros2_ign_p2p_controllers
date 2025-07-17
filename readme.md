# Forklift Control with ROS2 and Gazebo

This repository provides a solution for simulating and controlling a forklift model in Gazebo using ROS2. The project includes several key features that enhance the forklift’s simulation and control, ensuring a seamless integration into the Gazebo environment.


[![Watch the video](https://img.youtube.com/vi/H6YuKMIpOzQ?si=qNVGED4yRyyv_6P-/maxresdefault.jpg)](https://youtu.be/H6YuKMIpOzQ?si=qNVGED4yRyyv_6P-)
## Features

### 1. **Gazebo-Compatible Forklift URDF**
   - The forklift URDF model has been modified to ensure full compatibility with Gazebo Fortress.
   - Necessary ros2 plugins are integrated to handle sensors, actuators, and interactions within the simulation environment.


### 2. **Point-to-Point (P2P) Controller**
   - A controller is implemented to move the forklift from one point to another using a point-to-point (P2P) control strategy.

### 3. **Unit and Integration Testing**
   - Unit tests are implemented for control algorithms and system behavior to ensure reliability.
   - Integration tests validate the interaction between the forklift model, the controller, and the Gazebo simulation, confirming that all components work seamlessly together.

### 4. **Dockerized Setup**
   - The entire setup is containerized using Docker, providing a consistent and portable environment for easy deployment.
   - A Dockerfile is included for building the container and detailed instructions are provided for running the system within a Docker container.

## Setup and Installation

### Prerequisites
- **Docker**: Details on installing docker can be found [here](https://docs.docker.com/engine/install/).


### Instructions for running in docker

1. Clone the repository:
   
   
```
git clone https://github.com/ahmadabouelainein/ros2_ign_diffdrive/
```

2. To build the Docker image:
    
```
docker build -t p2p_forklift:nav ros2_ign_p2p_controllers/
```

3. To run the system in Docker. Note that this This would automatically run the launch file which opens gazebo with the forklift loaded in it:
    
```
docker run -it --rm --name p2p_container -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix p2p_forklift:nav
```

   
4. In another terminal run:
    
```
docker exec -it p2p_container bash
```

5. Once the terminal is attached to the container run:
    
```
source install/setup.bash && ros2 action send_goal /p2p_control nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 20.2, y: 10.1, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}
```

## License

Copyright © 2023 ahmadabouelainein \
This project is available under the terms of [the MIT License](LICENSE).
