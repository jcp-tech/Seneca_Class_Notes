FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    sudo curl gnupg2 lsb-release x11-apps && \
    useradd -m -u 1000 -s /bin/bash jetauto && \
    echo "jetauto ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && \
    apt-get install -y ros-melodic-desktop-full && \
    echo "source /opt/ros/melodic/setup.bash" >> /etc/skel/.bashrc

USER jetauto
WORKDIR /home/jetauto
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

CMD ["bash"]