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

## 🧪 Test TurtleSim (Lab 1)
### Run the following in separate terminals:

| Terminal       | Command                              |
| -------------- | ------------------------------------ |
| 🖥️ Terminal 1 | `roscore`                            |
| 🐢 Terminal 2  | `rosrun turtlesim turtlesim_node`    |
| 🎮 Terminal 3  | `rosrun turtlesim turtle_teleop_key` |


## 📁 Workspace & Package Dev (Lab 2 & 3)

The container comes pre-configured with a `catkin_ws`:
```bash
cd ~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
cd ..
catkin_make
source devel/setup.bash
```

Create a publisher node:
```bash
rosrun beginner_tutorials talker.py
```

And a subscriber node:
```bash
rosrun beginner_tutorials listener.py
```

### Common ROS Tools:
```bash
rosnode list
rostopic list
rosservice list
rosparam list
```

## 🧠 GPU & iGPU Detection (Auto)
- The `docker-compose.yml` will automatically try to use an NVIDIA GPU if available via the `runtime: nvidia` flag.
- If no NVIDIA GPU is detected, it still enables access to integrated GPUs (iGPU) via `/dev/dri` mapping.

## 🔧 Troubleshooting
- **DISPLAY Errors**: Ensure `xhost +local:` is run on host
- **No GUI**: Check if `DISPLAY` variable is set correctly (`echo $DISPLAY`)
- **NVIDIA Errors**: Check if `nvidia-container-toolkit` is installed