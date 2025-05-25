# 🐳 ROS Melodic Docker Setup for Seneca\_Class\_Notes

This setup builds a GPU- or CPU-enabled development environment for ROS Melodic using Docker.

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/jcp-tech/Seneca_Class_Notes.git
cd Seneca_Class_Notes
```

---

## 🏗️ Choose the Docker Compose Setup

### For GPU (NVIDIA GPU Support Required)

```bash
docker compose -f docker-compose.gpu.yml up --build
```

Ensure that:

* You have the latest **NVIDIA GPU drivers** installed
* The **NVIDIA Container Toolkit** is installed and configured

The GPU compose file includes:

```yaml
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

### For CPU-only (Integrated Graphics / No GPU)

```bash
docker compose -f docker-compose.cpu.yml up --build
```

This setup uses:

```yaml
    devices:
      - /dev/dri:/dev/dri
```

To provide access to direct rendering on CPU systems (e.g., Intel integrated graphics).

---

## 🖼️ GUI Support for ROS (e.g., `turtlesim`)

### On Linux:

```bash
export DISPLAY=:0
xhost +local:root
```

Then run your Docker container as above.

### On Windows 11 + WSLg:

GUI should work automatically (no DISPLAY config needed).

### To test GUI:

Once inside the container:

```bash
rosrun turtlesim turtlesim_node
```

> 🐢 A window should appear with the turtle simulator.

---

## 🧾 Additional Notes

* **User:** The Docker container runs as user `jetauto` for permissions compatibility.
* **Volumes:** The entire repo is mounted at `/workspace` inside the container.
* **Python Environment:** Python 3.12 is preinstalled and can be configured with packages inside the container.

---

## 🛠️ Helpful Commands

### Rebuild the Container (force new build):

```bash
docker compose -f docker-compose.gpu.yml build --no-cache
```

Or for CPU:

```bash
docker compose -f docker-compose.cpu.yml build --no-cache
```

### Stop and Remove Container:

```bash
docker stop ros-melodic && docker rm ros-melodic
```

### Enter the Container:

```bash
docker exec -it ros-melodic bash
```

✅ You now have a full Docker-based ROS Melodic setup with optional GPU or CPU support!