**Terminal:**

```bash
cd ~/jetauto_ws
sudo systemctl stop start_app_node.service
source devel/setup.bash
roslaunch lab5_jetauto_control jetauto_square.launch
```

<!-- 
    **Terminal 1:**
    ```bash
    cd ~/jetauto_ws
    sudo systemctl stop start_app_node.service
    source devel/setup.bash
    roslaunch lab5_jetauto_control custom_controller.launch
    ```

    **Terminal 2:**
    ```bash
    cd ~/jetauto_ws
    source devel/setup.bash
    rosrun lab5_jetauto_control jetauto_square.py
    ```
-->

---
---

### 1. `cd ~/jetauto_ws`

* **What it does:**
  Changes the current directory to your ROS workspace directory named `jetauto_ws` in your home directory.
* **Why:**
  Almost all ROS development and launch commands are run from within your catkin workspace, where your source code, packages, and build/devel folders live.

---

### 2. `sudo systemctl stop start_app_node.service`

* **What it does:**
  Stops a background service on your system named `start_app_node.service` using `systemctl` (the system and service manager for Linux).
* **Why:**
  This service probably auto-starts a ROS node or application on boot. You’re stopping it to avoid conflicts or to free up the robot’s ROS master, nodes, or resources before you launch your own code.

---

### 3. `source devel/setup.bash`

* **What it does:**
  Sets up the ROS environment for your current shell session by sourcing the `setup.bash` script found in your workspace’s `devel` folder.
* **Why:**
  This is essential so your shell knows about all ROS packages, dependencies, and environment variables you just built or added in your workspace. Without sourcing, ROS commands might not find your packages/nodes.

---

### 4. `roslaunch lab5_jetauto_control jetauto_square.launch`

* **What it does:**
  Launches a ROS launch file named `jetauto_square.launch` from the `lab5_jetauto_control` package.
* **Why:**
  Launch files are XML scripts that can start multiple ROS nodes, set parameters, and configure the robot’s behavior.
  This specific launch file likely starts the code to make the JetAuto robot move in a square pattern (using odometry or teleop), including initializing necessary controllers and node setups for lab 5.

---