FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    && curl -sSL 'http://packages.ros.org/ros.key' | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt update && apt install -y \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python3-pip \
    x11-apps

RUN rosdep init && rosdep update
RUN echo \"source /opt/ros/melodic/setup.bash\" >> /root/.bashrc

WORKDIR /workspace
CMD [\"bash\"]