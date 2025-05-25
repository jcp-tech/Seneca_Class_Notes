FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# --- Step 1: Basic tools + ROS repository ---
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    && curl -sSL "http://packages.ros.org/ros.key" | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# --- Step 2: Install ROS Melodic and dev tools ---
RUN apt update && apt install -y \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python3-pip \
    x11-apps

# --- Step 3: Install build deps for OpenCV, dlib, etc. ---
RUN apt install -y \
    cmake \
    libboost-all-dev \
    libgtk2.0-dev \
    libcanberra-gtk-module

# --- Step 4: Initialize ROS ---
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]