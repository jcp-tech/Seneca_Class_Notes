
# 🌐 Connect to Ubuntu 20 via SSH with Docker (Ubuntu 18.04 Container)

This guide outlines a detailed and organized setup for using Docker to run ROS Melodic and TurtleSim on a remote Ubuntu 20.04 system using an Ubuntu 18.04 container.

---

## ✅ Prerequisites (On the Host Ubuntu System)

- Ubuntu 20.04 with GUI
- Docker installed: [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
- Docker Compose installed: [Install Docker Compose](https://docs.docker.com/compose/install/)
- X11 display forwarding enabled:
  ```bash
  xhost +SI:localuser:$(whoami)
  ```

### 🖥️ For GPU Users:
- Latest NVIDIA drivers installed (verify with `nvidia-smi`)
- `nvidia-docker2` and `nvidia-container-toolkit` installed
- Test GPU access:
  ```bash
  docker run --gpus all nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi
  ```

---

## 🧹 Clean Existing Docker Containers & Images

```bash
docker container stop $(docker ps -aq)
docker container rm $(docker ps -aq)
docker image rm ros-melodic-dev-cpu ros-melodic-dev-gpu
```

---

## 🚀 Build and Run ROS Docker Containers

1. **Build Docker Images**
   ```bash
   docker compose build
   ```

2. **Start Docker Containers**
   ```bash
   docker compose up -d
   ```

---

## ▶️ Access and Run ROS in Container

1. **Enter the container:**
   ```bash
   docker exec -it ros-melodic-container-gpu bash
   ```

2. **Verify Ubuntu version and ROS installation:**
   ```bash
   lsb_release -a
   source /opt/ros/melodic/setup.bash
   ```

---

## 🖼️ Test Additional GUI Apps

```bash
docker exec -it ros-melodic-container-cpu bash
source /opt/ros/melodic/setup.bash
xeyes
```

---

## 🔧 Troubleshooting

- **DISPLAY Errors:** Don't use `ssh -X`. Instead, set `export DISPLAY=:0` manually in container.
- **No GUI on host display:** Run `xhost +SI:localuser:jetauto` on host.
- **NVIDIA Errors:** Ensure toolkit is installed and run:
  ```bash
  docker run --gpus all nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi
  ```

---

## 🧾 Verified ROS Environment Inside Container

- Ubuntu 18.04
- ROS Melodic Desktop-Full
- ROS Tools: rosdep, rosinstall, rosinstall-generator, wstool
- GUI Tools: turtlesim, rqt, rqt-common-plugins
- ROS automatically sourced in `.bashrc`
- Host GUI display forwarding enabled

---

## 🧾 Example SSH Session Log (Container Startup)

```bash
cd 'Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/Docker/'
```
