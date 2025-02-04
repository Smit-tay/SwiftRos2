# Start with an official ROS 2 base image for the desired distribution
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    bash-completion \
    curl \
    gdb \
    git \
    nano \
    openssh-client \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    sudo \
    vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /swiftros_ws

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /swiftros_ws

USER $USERNAME

# Install some ROS 2 dependencies to create a cache layer
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2controlcli \
    ros-humble-controller-interface \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-controller-manager \
    ros-humble-ros2-control-test-assets \
    ros-humble-hardware-interface \
    ros-humble-control-msgs \
    ros-humble-generate-parameter-library \
    ros-humble-realtime-tools \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-visualization \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-rviz2 \
    ros-humble-xacro \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Install the missing ROS 2 dependencies
COPY . /swiftros_ws/src
RUN sudo chown -R $USERNAME:$USERNAME /swiftros_ws \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/.ros 

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT []
WORKDIR /swiftros_ws
