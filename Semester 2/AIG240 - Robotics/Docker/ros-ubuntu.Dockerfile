FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

# Basic Tools | Unique to ROS-Ubuntu
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release sudo x11-apps vim net-tools python-pip && \
    rm -rf /var/lib/apt/lists/*

# Create user
RUN useradd -m -u 1000 -s /bin/bash jetauto && \
    echo "jetauto ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add ROS sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS + Core Dev Tools
RUN apt-get update && \
    apt-get install -y ros-melodic-desktop-full \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    ros-melodic-turtlesim ros-melodic-rqt ros-melodic-rqt-common-plugins \
    gazebo9 libgazebo9-dev && \
    rm -rf /var/lib/apt/lists/*

# Install ROS simulation + visualization tools
RUN apt-get update && apt-get install -y \
    ros-melodic-joint-state-publisher \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-robot-state-publisher \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    ros-melodic-xacro \
    ros-melodic-rviz \
    ros-melodic-controller-manager \
    ros-melodic-transmission-interface \
    ros-melodic-effort-controllers \
    ros-melodic-position-controllers \
    ros-melodic-velocity-controllers && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# SLAM + Navigation stack
RUN apt-get update && apt-get install -y \
    ros-melodic-slam-gmapping \
    ros-melodic-hector-slam \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-amcl \
    ros-melodic-navfn \
    ros-melodic-costmap-2d \
    ros-melodic-global-planner \
    ros-melodic-base-local-planner \
    ros-melodic-teb-local-planner \
    ros-melodic-dwa-local-planner \
    ros-melodic-robot-localization \
    ros-melodic-rqt-robot-steering && \
    rm -rf /var/lib/apt/lists/*

# Install Gazebo and additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-melodic-gazebo-ros-control \
    ros-melodic-urdf \
    ros-melodic-urdf-tutorial \
    ros-melodic-xacro \
    ros-melodic-joint-state-publisher \
    ros-melodic-robot-state-publisher \
    ros-melodic-urdfdom-py \
    ros-melodic-rviz \
    ros-melodic-rqt-robot-steering \
    ros-melodic-rqt-graph \
    ros-melodic-rqt-console \
    ros-melodic-rqt-gui \
    ros-melodic-rqt-common-plugins \
    ros-melodic-ros-control \
    ros-melodic-controller-manager \
    ros-melodic-diff-drive-controller \
    ros-melodic-robot-localization \
    ros-melodic-tf \
    ros-melodic-tf2 \
    ros-melodic-roslaunch \
    ros-melodic-turtlesim \
    ros-melodic-teleop-twist-keyboard \
    openssh-server \
    net-tools \
    iputils-ping \
    iproute2 \
    xauth \
    python-pip \
    python3-pip \
    python3-pynput \
    python-catkin-tools \
    build-essential \
    nano vim tmux screen \
    wget curl git lsb-release sudo \
    && apt-get clean

# Switch to user
USER jetauto
WORKDIR /home/jetauto

# ARG for custom catkin workspace directory (path to the folder containing devel/setup.bash)
ARG CUSTOM_CATKIN_WS_DIR=""

# Auto-source ROS and then custom catkin workspace
RUN { \\\
        echo '# Source ROS main setup'; \\\
        echo 'source /opt/ros/melodic/setup.bash'; \\\
        echo '# Check and source custom catkin workspace'; \\\
        echo 'CUSTOM_WS_DIR_ENV="'"$CUSTOM_CATKIN_WS_DIR"'"'; \\\
        echo 'if [ -n "$CUSTOM_WS_DIR_ENV" ] && [ -f "$CUSTOM_WS_DIR_ENV/devel/setup.bash" ]; then'; \\\
        echo '  echo "Sourcing custom catkin workspace from $CUSTOM_WS_DIR_ENV/devel/setup.bash"'; \\\
        echo '  source "$CUSTOM_WS_DIR_ENV/devel/setup.bash"'; \\\
        echo 'elif [ -n "$CUSTOM_WS_DIR_ENV" ]; then'; \\\
        echo '  echo "Custom catkin workspace configured at $CUSTOM_WS_DIR_ENV, but devel/setup.bash not found."'; \\\
        echo 'fi'; \\\
    } >> /home/jetauto/.bashrc

RUN mkdir -p /var/run/sshd
RUN echo 'jetauto:jetauto' | chpasswd

RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config

EXPOSE 22

ENV DISPLAY=:0

CMD ["bash"]