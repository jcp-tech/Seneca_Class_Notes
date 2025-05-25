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

## 🧪 Run TurtleSim (Lab 1)

| Terminal       | Command                              |
| -------------- | ------------------------------------ |
| 🖥️ Terminal 1 | `docker exec -it ros-melodic-container bash`<br>`roscore` |
| 🐢 Terminal 2  | `docker exec -it ros-melodic-container bash`<br>`rosrun turtlesim turtlesim_node` |
| 🎮 Terminal 3  | `docker exec -it ros-melodic-container bash`<br>`rosrun turtlesim turtle_teleop_key` |

## 🔧 Troubleshooting
- **DISPLAY Errors**: Ensure `xhost +local:` is run on host
- **No GUI**: Check if `DISPLAY` variable is set correctly (`echo $DISPLAY`)
- **NVIDIA Errors**: Check if `nvidia-container-toolkit` is installed