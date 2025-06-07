FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

# Basic Tools
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

# Switch to user
USER jetauto
WORKDIR /home/jetauto

# Source both ROS and your external workspace if mounted
RUN echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc && \
    echo 'if [ -f "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/devel/setup.bash" ]; then' >> ~/.bashrc && \
    echo '  source "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/devel/setup.bash"' >> ~/.bashrc && \
    echo 'fi' >> ~/.bashrc

CMD ["bash"]
