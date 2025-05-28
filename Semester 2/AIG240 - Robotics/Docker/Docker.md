# Docker.md

## ✅ Prerequisites

Before using Docker for ROS and TurtleSim, ensure the following are set up **on the host Ubuntu system**:

* Ubuntu 18.04 Desktop (GUI-enabled)
* Docker installed: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
* Docker Compose installed: [https://docs.docker.com/compose/install/](https://docs.docker.com/compose/install/)
* X11 display forwarding configured:

  ```bash
  xhost +SI:localuser:$(whoami)
  ```
* For GPU usage:

  * Latest NVIDIA drivers installed
  * `nvidia-docker2` and `nvidia-container-toolkit` installed
  * Verify with: `docker run --gpus all nvidia/cuda:11.0-base nvidia-smi`

## 🧹 Clean Existing Docker Containers & Images

```bash
docker container stop $(docker ps -aq)
docker container rm $(docker ps -aq)
docker image rm ros-melodic-dev-cpu ros-melodic-dev-gpu
```

## 🚀 Build the Docker Images

```bash
docker compose build
```

## ▶️ Run the Containers

```bash
docker compose up -d
```

## 🧪 Run TurtleSim (Lab 1 Verified Setup)

| Terminal       | Command                                                                                  |
| -------------- | ---------------------------------------------------------------------------------------- |
| 🖥️ Terminal 1 | `docker exec -it ros-melodic-container-cpu bash`<br>`roscore`                            |
| 🐢 Terminal 2  | `docker exec -it ros-melodic-container-cpu bash`<br>`rosrun turtlesim turtlesim_node`    |
| 🎮 Terminal 3  | `docker exec -it ros-melodic-container-cpu bash`<br>`rosrun turtlesim turtle_teleop_key` |

## 🐢 Test Additional GUI Apps

```bash
docker exec -it ros-melodic-container-cpu bash
source /opt/ros/melodic/setup.bash
rqt  # or
xeyes
```

## 🔧 Troubleshooting

* **DISPLAY Errors**: Ensure you're not SSHing with `-X`; use the native terminal
* **No GUI on Ubuntu screen**: Run `export DISPLAY=:0` in container if not inherited
* **Permission Denied**: On Ubuntu, run: `xhost +SI:localuser:jetauto`
* **NVIDIA Errors**: Ensure `nvidia-container-toolkit` is installed, and container uses `runtime: nvidia`

## 🧾 Verified ROS Setup Includes:

* Ubuntu 18.04
* ROS Melodic Desktop-Full
* `rosdep`, `rosinstall`, `rosinstall-generator`, `wstool`
* `ros-melodic-turtlesim`
* `ros-melodic-rqt` and `rqt-common-plugins`
* ROS source in `.bashrc`
* GUI forwarding enabled for physical Ubuntu display

---

✅ This setup **fully satisfies the Lab 1 PDF** requirements for ROS + TurtleSim on both CPU and GPU containers.