FROM nvidia/cuda:11.8.0-base-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# Create user
RUN apt-get update && apt-get install -y \
    sudo curl gnupg2 lsb-release x11-apps vim net-tools && \
    useradd -m -u 1000 -s /bin/bash jetauto && \
    echo "jetauto ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add ROS repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS and build tools
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    ros-melodic-turtlesim ros-melodic-rqt ros-melodic-rqt-common-plugins

# Initialize rosdep
RUN rosdep init && rosdep update && rosdep update --rosdistro=melodic

# Final setup
USER jetauto
WORKDIR /home/jetauto
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

CMD ["bash"]