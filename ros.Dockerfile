FROM ubuntu:18.04

# Install ROS Melodic
RUN apt update && apt install -y curl gnupg2 lsb-release \
 && curl -sSL 'http://packages.ros.org/ros.key' | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list \
 && apt update && apt install -y ros-melodic-ros-base python3-pip x11-apps

# Setup ROS environment
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Additional ROS tools
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

WORKDIR /workspace
CMD ["bash"]