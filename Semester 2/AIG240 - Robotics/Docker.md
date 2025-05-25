# 🐳 ROS Melodic Docker Setup for Seneca\_Class\_Notes

This setup builds a GPU-enabled development environment for ROS Melodic, accessible via Docker.

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/jcp-tech/Seneca_Class_Notes.git
cd Seneca_Class_Notes
```

### 2. Build the Docker Image

Run this from inside the cloned repo:

```bash
sudo docker compose build --no-cache
```

### 🖼️ GUI Support for ROS (e.g., `turtlesim`)

If you're planning to run ROS GUI tools (like `rqt` or `turtlesim`), ensure that the `DISPLAY` variable is set **before launching Docker**.

#### 🧭 For Linux/X11:

```bash
export DISPLAY=:0
```

Then:

```bash
DISPLAY=:0 docker compose up
```

#### 🪟 On Windows 11 using WSLg:

GUI apps should work automatically — no need to set DISPLAY manually.

#### 🧪 To test:

Inside the container, run:

```bash
rosrun turtlesim turtlesim_node
```

> ✅ A window should appear with the turtle GUI.

---

### 3. Run the Docker Container

```bash
./ros_docker_run.sh
```

Or, run the script directly:

```bash
./scripts/auto_attach_docker.sh
```

This will auto-start the container (if needed) and attach to it.

---

## 🧾 Additional Notes

* **User:** The Docker container runs under user `jetauto` for permission consistency.
* **Volumes:** The entire repo is mounted at `/workspace` inside the container.
* **Environment:** Python 3.12 and base ML libraries (TensorFlow, PyTorch, etc.) can be installed inside or outside the container depending on your workflow.

---

## 🛠️ Helpful Commands

### To Rebuild the Container:

```bash
sudo docker compose build --no-cache
```

### To Stop and Remove Container:

```bash
docker stop ros-melodic && docker rm ros-melodic
```

### To Enter the Container Manually:

```bash
docker exec -it ros-melodic bash
```

---

For any additional setup or to install missing packages inside the container, edit `ros.Dockerfile` or run commands after `docker exec`.