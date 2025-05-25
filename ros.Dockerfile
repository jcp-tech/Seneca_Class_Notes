FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# --- Step 1: Install system and ROS dependencies ---
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    build-essential \
    python3-pip \
    x11-apps \
    cmake \
    libboost-all-dev \
    libgtk2.0-dev \
    libcanberra-gtk-module \
    sudo

# --- Step 2: Set up ROS repository and install ROS ---
RUN curl -sSL "http://packages.ros.org/ros.key" | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt update && apt install -y \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool

# --- Step 3: Initialize ROS ---
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# --- Step 4: Add non-root user ---
ARG USERNAME=jetauto
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo $USERNAME

# --- Step 5: Install Python packages via pip with caching ---
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --upgrade pip && \
    pip3 install -r /tmp/requirements.txt

# --- Step 6: Switch to user and workspace directory ---
USER $USERNAME
WORKDIR /workspace

CMD ["bash"]