FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language support
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install ROS2
RUN apt-get update  && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl http://repo.ros2.org/repos.key | apt-key add -  && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'  && \
    apt-get update && \
    apt-get install -y ros-humble-desktop python3-argcomplete && \
    apt-get install -y python3-pip python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

FROM base AS dev

RUN apt-get update && \
    apt-get install -y bash-completion  && \
    apt-get install -y git && \
    apt-get install -y build-essential cmake gdb && \
    apt-get install -y pylint python3-argcomplete && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install -y python3-vcstool && \
    apt-get install -y wget && \
    apt-get install -y ros-humble-tf-transformations && \
    apt-get install -y ros-humble-ament-lint python3-autopep8 && \
    apt-get install -y ros-humble-launch-testing ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros && \
    apt-get install -y nano && \
    apt-get install -y htop && \
    apt-get install -y python3-bloom python3-rosdep fakeroot && \
    apt-get install -y dpkg-dev debhelper && \
    apt-get install -y lcov && \
    apt-get install -y ros-humble-control-msgs && \
    apt-get install -y ros-humble-hardware-interface && \
    apt-get install -y ros-humble-joint-state-broadcaster && \
    apt-get install -y ros-humble-joint-trajectory-controller && \
    apt-get install -y ros-humble-ros-ign-gazebo && \
    apt-get install -y ros-humble-ros2controlcli && \
    apt-get install -y ros-humble-xacro && \
    apt-get install -y ros-humble-effort-controllers && \
    apt-get install -y ros-humble-imu-sensor-broadcaster && \
    apt-get install -y ros-humble-ros-gz-bridge && \
    apt-get install -y ros-humble-velocity-controllers && \
    apt-get install -y ros-humble-diff-drive-controller && \
    apt-get install -y ros-humble-tricycle-controller && \
    apt-get install -y ros-humble-ackermann-steering-controller && \
    apt-get install -y libignition-gazebo6-dev && \
    apt-get install -y libignition-plugin-dev && \
    apt-get install -y ros-humble-controller-manager && \
    apt-get install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup && \
    apt-get update && \
    apt-get install -y ros-humble-gazebo-ros-pkgs && \
    pip3 install colcon-mixin && \
    pip3 install colcon-common-extensions && \
    pip3 install colcon-lcov-result && \
    pip3 install ipykernel && \
    pip3 install matplotlib && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

ARG USERNAME=ahmad
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get update && \
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Cleanup
RUN apt-get update \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
ARG WORKSPACE=/ws/ros2_ign_diffdrive
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    mkdir -p ${WORKSPACE}/src && cd ${WORKSPACE} &&\
    git clone -b feature/containerize https://github.com/ahmadabouelainein/ros2_ign_diffdrive && \
    cd ${WORKSPACE}/src && sudo rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

SHELL [ "/bin/bash" , "-c" ]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${WORKSPACE}/ && \
    colcon build
ENV USERNAME=${USERNAME}    
WORKDIR ${WORKSPACE}
CMD ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE/install/setup.bash && exec bash"]
ENTRYPOINT /bin/bash