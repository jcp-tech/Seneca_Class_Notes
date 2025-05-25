FROM ubuntu:18.04

# Avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Create user jetauto with UID and GID 1000
RUN apt-get update && \
    apt-get install -y sudo && \
    useradd -m -u 1000 -s /bin/bash jetauto && \
    echo "jetauto ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install ROS Melodic and tools
RUN apt-get update && \
    apt-get install -y gnupg2 curl && \
    curl -sSL http://packages.ros.org/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && \
    apt-get install -y ros-melodic-desktop-full \
    build-essential git wget cmake x11-apps && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/melodic/setup.bash" >> /etc/skel/.bashrc

# Install ROS dependencies for building packages
RUN apt-get install -y python-rosinstall python-rosinstall-generator python-wstool python-catkin-tools

# Switch to user jetauto
USER jetauto
WORKDIR /workspace

# Source ROS in bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

CMD ["bash"]