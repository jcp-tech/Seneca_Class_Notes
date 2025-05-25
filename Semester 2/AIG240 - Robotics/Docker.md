# 🐳 ROS Melodic Docker Setup - Quick Start Guide

This guide explains how to build, run, and access the ROS Melodic Docker container used in this repository.

---

## 📁 Folder Structure

```
Seneca_Class_Notes/
├── ros.Dockerfile
├── docker-compose.yml
├── scripts/
│   └── auto_attach_docker.sh
└── ros_docker_run.sh
```

---

## 🚀 Initial Setup (First Time Only)

### 🔧 Step 1: Build the Docker Image

```bash
cd ~/Desktop/Seneca_Class_Notes
sudo docker compose build --no-cache
```

> This may take 10-15 minutes on the first build. Make sure you are connected to the internet.

---

## 🔄 Run and Access Docker

### ▶️ Step 2: Start the Container

```bash
sudo docker compose up -d
```

### 🔗 Step 3: Attach to the Container

#### Option A: Recommended (alias or launcher)

```bash
./ros_docker_run.sh
```

#### Option B: Manual

```bash
docker exec -it ros-melodic bash
```

---

## 🐢 Run ROS

Once inside the container, try:

```bash
roscore
```

If you see the master starting with no errors, ROS is working correctly.

---

## 🧠 Notes

* The container mounts the entire project directory to `/workspace`
* The container auto-loads ROS setup in `.bashrc`
* The default user is `jetauto` with **no password required for sudo**
* GUI (turtlesim) is enabled via X11 if your host supports it

---

## 💡 Tips

* Add an alias to your `~/.bashrc` to make it easier:

```bash
alias rosdock='~/ros_docker_run.sh'
```

* Use `docker compose down` to stop the container completely
* Add the script `ros_docker_run.sh` to your PATH for global access

---

## ✅ Done

You’re now ready to use ROS Melodic in a fully containerized setup with GPU, sudo access, GUI support, and persistent workspace!

Feel free to add more launch scripts or simulation packages as needed.
