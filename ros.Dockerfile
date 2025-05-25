FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# --- Step 1: System dependencies ---
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    build-essential \
    wget \
    git \
    make \
    cmake \
    sudo \
    x11-apps \
    libboost-all-dev \
    libgtk2.0-dev \
    libcanberra-gtk-module

# --- Step 2: ROS repository and install ---
RUN curl -sSL "http://packages.ros.org/ros.key" | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt update && apt install -y \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool

# --- Step 3: ROS setup ---
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# --- Step 4: Create non-root user ---
ARG USERNAME=jetauto
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/melodic/setup.bash" >> /home/$USERNAME/.bashrc

# --- [Optional] Step 5: Install Python 3.12 and ML packages (commented out) ---
# RUN cd /usr/src && \
#     wget https://www.python.org/ftp/python/3.12.3/Python-3.12.3.tgz && \
#     tar xzf Python-3.12.3.tgz && \
#     cd Python-3.12.3 && \
#     ./configure --enable-optimizations && \
#     make -j$(nproc) && \
#     make altinstall && \
#     ln -sf /usr/local/bin/python3.12 /usr/bin/python3 && \
#     ln -sf /usr/local/bin/pip3.12 /usr/bin/pip3

# COPY requirements.txt /tmp/requirements.txt
# RUN pip3 install --upgrade pip && \
#     pip3 install -r /tmp/requirements.txt

USER $USERNAME
WORKDIR /workspace
CMD ["bash"]