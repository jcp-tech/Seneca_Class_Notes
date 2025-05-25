FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt update && apt install -y \
    curl gnupg2 lsb-release tzdata sudo \
    build-essential x11-apps libboost-all-dev \
    libgtk2.0-dev libcanberra-gtk-module \
    python-rosdep python-rosinstall python-rosinstall-generator \
    python-wstool cmake wget git \
    && curl -sSL "http://packages.ros.org/ros.key" | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt update && apt install -y ros-melodic-desktop-full \
    && rosdep init && rosdep update \
    && echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

ARG USERNAME=jetauto
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME

USER $USERNAME
WORKDIR /workspace

CMD ["bash"]