FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# --- Step 1: ROS source setup ---
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    && curl -sSL "http://packages.ros.org/ros.key" | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# --- Step 2: ROS and development tools ---
RUN apt update && apt install -y \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python3-pip \
    x11-apps \
    cmake \
    libboost-all-dev \
    libgtk2.0-dev \
    libcanberra-gtk-module

# --- Step 3: ROS setup ---
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# --- Step 4: Create non-root user (jetauto) with same UID:GID as host ---
ARG USERNAME=jetauto
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo $USERNAME \

USER $USERNAME
WORKDIR /workspace

CMD ["bash"]