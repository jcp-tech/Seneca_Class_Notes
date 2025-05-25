# Docker.md

## 🚀 Build the Docker Image
```bash
docker compose build
```

## ▶️ Run the Container
```bash
docker compose up -d
```

## 🐢 Test ROS GUI Apps
```bash
docker exec -it ros-melodic-container bash
source /opt/ros/melodic/setup.bash
rosrun turtlesim turtlesim_node
```
Make sure your host allows X11 forwarding. On Linux:
```bash
xhost +local:
```

## 🧠 GPU & iGPU Detection (Auto)
- The `docker-compose.yml` will automatically try to use an NVIDIA GPU if available via the `runtime: nvidia` flag.
- If no NVIDIA GPU is detected, it still enables access to integrated GPUs (iGPU) via `/dev/dri` mapping.

## 🔧 Troubleshooting
- **DISPLAY Errors**: Ensure `xhost +local:` is run on host
- **No GUI**: Check if `DISPLAY` variable is set correctly (`echo $DISPLAY`)
- **NVIDIA Errors**: Check if `nvidia-container-toolkit` is installed