FROM ros:melodic-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# TEMPORARILY remove broken ROS repo and fix expired key | Unique to ROS-Official
RUN sed -i '/snapshots.ros.org/d' /etc/apt/sources.list /etc/apt/sources.list.d/* || true && \
    apt-get update && apt-get install -y curl gnupg2 lsb-release sudo x11-apps vim net-tools python-pip && \
    apt-key del F42ED6FBAB17C654 || true && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Create user
RUN useradd -m -u 1000 -s /bin/bash jetauto && \
    echo "jetauto ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add ROS repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS and tools
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    ros-melodic-turtlesim ros-melodic-rqt ros-melodic-rqt-common-plugins \
    gazebo9 libgazebo9-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up environment
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
    python-catkin-tools \
    build-essential \
    nano vim tmux screen \
    wget curl git lsb-release sudo \
    && apt-get clean

# Then ADD this:
RUN apt-get update && apt-get install -y python-pip python3-pip
RUN pip install --no-cache-dir pynput==1.6.3
RUN python3 -m pip install --no-cache-dir pynput==1.6.3

# Initialize rosdep
RUN rosdep init && rosdep update && rosdep update --rosdistro=melodic

# -- SSH configuration
RUN mkdir -p /var/run/sshd && \
    echo 'jetauto:jetauto' | chpasswd && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config

# -- Switch to user
USER jetauto
WORKDIR /home/jetauto

# -- ARG for custom catkin workspace directory (path to the folder containing devel/setup.bash)
ARG CUSTOM_CATKIN_WS_DIR=""

# -- Auto-source ROS and custom catkin workspace
RUN echo "if [ -f \"$CUSTOM_CATKIN_WS_DIR/devel/setup.bash\" ]; then source \"$CUSTOM_CATKIN_WS_DIR/devel/setup.bash\"; fi" >> /home/jetauto/.bashrc
# RUN echo "source /mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/devel/setup.bash" >> /home/jetauto/.bashrc

EXPOSE 22
ENV DISPLAY=:0
CMD ["bash"]