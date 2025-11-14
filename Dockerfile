# Choose from humble and iron
ARG FROM_IMAGE=ros:jazzy
FROM $FROM_IMAGE

ARG OVERLAY_WS=/opt/ros/overlay_ws
# Choose from fortress, garden, and harmonic.
ARG GZ_VERSION=harmonic
ENV GZ_VERSION=${GZ_VERSION}

# Install basic tools first (lightweight, changes rarely)
RUN apt-get update \
    && apt-get install -y lsb-release wget gnupg git \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic (HEAVY - cache this layer)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y "gz-${GZ_VERSION}"  \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages and desktop (HEAVY - cache this layer)
RUN apt-get update \
    && apt-get install -y \
        "ros-${ROS_DISTRO}-ros-gz" \
        "ros-${ROS_DISTRO}-desktop" \
        "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
        "ros-${ROS_DISTRO}-turtlebot4-simulator" \
        "ros-${ROS_DISTRO}-turtlebot4-navigation" \
        "ros-${ROS_DISTRO}-turtlebot4-description" \
        "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
        "ros-${ROS_DISTRO}-irobot-create-msgs" \
        libgl1-mesa-dri \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages (medium weight)
ARG PIP_BREAK_SYSTEM_PACKAGES=TRUE
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Install development and convenience tools (lightweight, may change)
RUN apt-get update \
    && apt-get install -y \
        nano vim htop tmux screen \
        bash-completion sudo zsh ros-${ROS_DISTRO}-rosbridge-server libwebsocketpp-dev libboost-system-dev ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Create ros user with sudo access (lightweight, may change during development)
RUN groupadd -g 1000 ros || groupmod -n ros $(getent group 1000 | cut -d: -f1) \
    && useradd -m -u 1000 -g 1000 -s /bin/bash ros || usermod -l ros $(getent passwd 1000 | cut -d: -f1) \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

# Ensure workspace directory exists and has correct permissions
RUN mkdir -p /workspace && chown -R 1000:1000 /workspace

USER ros

CMD ["/bin/bash"]
