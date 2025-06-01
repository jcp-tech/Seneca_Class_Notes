
# 🧪 Lab 1: ROS Melodic Setup (Applicable to Both WSL and Docker Systems)

This section outlines the common setup and execution steps for Lab 1 using ROS Melodic and TurtleSim.

---

## 🔧 Step-by-Step Setup Instructions

1. **Create Workspace:**
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   ```

2. **Source Workspace in .bashrc:**
   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Check ROS Environment Variables:**
   ```bash
   echo $ROS_DISTRO
   echo $ROS_PACKAGE_PATH
   ```

4. **Install turtlesim (if not installed):**
   ```bash
   sudo apt install ros-melodic-turtlesim -y
   ```

---

## ▶️ Launch ROS Master and Nodes

- 🖥️ **Terminal 1:** Start ROS Master
   ```bash
   roscore
   ```

- 🐢 **Terminal 2:** Launch TurtleSim GUI
   ```bash
   rosrun turtlesim turtlesim_node
   ```

- 🎮 **Terminal 3:** Control TurtleSim with keyboard
   ```bash
   rosrun turtlesim turtle_teleop_key
   ```

> 📝 Make sure Terminal 3 is **focused and active** when pressing arrow keys.

---

## 📡 Verify ROS Functionality

- List all active nodes:
   ```bash
   rosnode list
   ```

- List available topics:
   ```bash
   rostopic list
   ```

- Echo pose updates from turtle1:
   ```bash
   rostopic echo /turtle1/pose
   ```

---

### 💻 Your Terminal Setup Should Look Like:

| Terminal       | Command                              |
| -------------- | ------------------------------------ |
| 🖥️ Terminal 1 | `roscore`                            |
| 🐢 Terminal 2  | `rosrun turtlesim turtlesim_node`    |
| 🎮 Terminal 3  | `rosrun turtlesim turtle_teleop_key` |

> 📝 Make sure Terminal 3 is **focused and active** when pressing arrow keys.
