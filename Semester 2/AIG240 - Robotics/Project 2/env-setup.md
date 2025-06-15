# **JetAuto Project & Simulation: Full Environment Setup Documentation**

---

## **A. Original Ubuntu System with Docker (as done in Lab4 & Project 2)**

---

### **1. Navigate to Docker Compose Directory**

Navigate to the directory containing your `docker-compose.yml` file. This is your project’s root for container builds and orchestration.

---

### **2. (Re)build Docker Containers**

Rebuild your Docker containers to ensure all dependencies and configurations are up to date.
You may keep multiple Dockerfile/container variations for experimentation and troubleshooting.

---

### **3. Download Workspace Archive**

Download the JetAuto workspace zip file from:
[https://drive.google.com/file/d/1SSaqoji\_3H5vm9ZKAljeWPFmDUGfCc\_u/view?usp=drive\_link](https://drive.google.com/file/d/1SSaqoji_3H5vm9ZKAljeWPFmDUGfCc_u/view?usp=drive_link)

---

### **4. Extract the Workspace to Your Repository**

Unzip the downloaded archive into your project repository.
This ensures your workspaces remain organized in a single, easily accessible folder.

---

### **5. Create a Symbolic Link to `/home/jetauto/jetauto_ws/`**

Create a symbolic link from your workspace in the repository to the default ROS path:

```bash
ln -s "/home/jetauto/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/jetauto_ws" /home/jetauto/jetauto_ws
```

This enables you to work from either directory seamlessly.

---

### **6. Clean Restart of Docker Containers**

Shut down any running containers, remove them for a clean state, and rebuild them to ensure a fresh environment.
Use:
Enter the `docker-compose.yml` Location

```bash
cd '/home/jetauto/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/Docker'
```

Rebuild the Container

```bash
docker compose down
# Optionally remove containers
docker compose build
docker compose up -d
```

---

### **7. Enter the Docker Container**

Access your running container interactively to perform workspace operations and testing.

```bash
docker exec -it ros_ubuntu_container bash
```

or my backup Container Build

```bash
docker exec -it ros_official_container bash
```

---

### **8. Test ROS Launch Files**

Launch RViz using your JetAuto description package to verify that ROS and visualization tools are functioning:

```bash
roslaunch jetauto_description display.launch model:=urdf/jetauto.urdf
```

---

### **9. Update Shebangs in All Python Files**

Since you’re running Ubuntu 18.04 with ROS Melodic (which defaults to Python 2), update the shebangs in all Python scripts to `#!/usr/bin/env python`.

```bash
find . -type f -name "*.py" -exec sed -i '1s|^#!/usr/bin/env python3$|#!/usr/bin/env python|' {} +
```

---

### **10. Make All Python Scripts Executable**

Grant executable permissions to all `.py` files throughout the workspace:

```bash
find ~/jetauto_ws/src -type f -name "*.py" -exec chmod +x {} +
```

---

### **11. Launch Gazebo Simulation**

Start the Gazebo environment and verify the robot spawns correctly:

```bash
roslaunch jetauto_gazebo worlds.launch
```

---

### **12. Test Teleoperation via Keyboard**

Ensure manual control is functional with:

```bash
roslaunch jetauto_peripherals teleop_key_control.launch
```

Use WASD keys to drive the robot.

---

> **Note:** All environment variables, Python libraries, and APT dependencies are preconfigured in the [Docker build files](https://github.com/jcp-tech/Seneca_Class_Notes/tree/master/Semester%202%2FAIG240%20-%20Robotics%2FDocker).

---

### **13. Create Python Package for Project 2**

* a. Create the Project 2 control package:

  ```bash
  catkin_create_pkg project2_jetauto_control rospy std_msgs geometry_msgs
  ```
* b. Also create the Lab 4 control package:

  ```bash
  catkin_create_pkg lab4_jetauto_control rospy std_msgs geometry_msgs
  ```
* c. Delete the default `src` folder from the lab4 package.
* d. Create a `scripts` folder in the project2 package.
* e. Add symbolic links from project2’s `src` and `scripts` folders to lab4, ensuring code sharing between packages.
* f. Create the control Python file (e.g., `jetauto_control.py`) in the appropriate location.
* g. Make the new script executable with `chmod +x`.

---

### **14. Build the Workspace**

Attempt to build the workspace:

```bash
catkin_make
```

If you encounter issues (e.g., workspace previously built with `catkin build`), use:

```bash
catkin build
```

Some error messages may appear, but as long as new folders and scripts are built, proceed.

---

### **15. Develop and Refine Code**

Iterate on your code until the desired functionality is achieved and tested.

---

### **16. Run the Simulation and Node**

**Terminal 1:**

```bash
cd ~/jetauto_ws
source devel/setup.bash
roslaunch jetauto_gazebo worlds.launch
```

**Terminal 2:**

```bash
cd ~/jetauto_ws
source devel/setup.bash
rosrun project2_jetauto_control jetauto_square.py
```

---

### **17. Verify Successful Operation in Docker**

Confirm that your Project 2 script works in the Ubuntu 20.04 Docker container.

---

### **18. Commit All Changes**

Commit all modifications in `jetauto_ws/src/` and any additional relevant files to your version control system (e.g., git).

---

---

## **B. Setting Up on WSL (Windows Subsystem for Linux)**

---

---

### **19. Complete WSL Setup for JetAuto Simulation**
>  Taken from the `.Dockerfile` to Update my WSL Ubuntu.
* Prepare the workspace by ensuring all required files are present and creating a symbolic link for ease of access:

  ```bash
  ln -s "/mnt/c/Users/JonathanChackoPattas/OneDrive - Maritime Support Solutions/Desktop/Class Notes/Seneca/Semester 2/AIG240 - Robotics/jetauto_ws" /home/jetauto/jetauto_ws
  ```

* Clean up any broken or expired ROS sources and update to the latest keys:

  ```bash
  sudo sed -i '/snapshots.ros.org/d' /etc/apt/sources.list /etc/apt/sources.list.d/* || true
  sudo apt-get update
  sudo apt-get install -y curl gnupg2 lsb-release sudo x11-apps vim net-tools python-pip
  sudo apt-key del F42ED6FBAB17C654 || true
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
  echo "deb http://packages.ros.org/ros/ubuntu bionic main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
  ```

* Add a modern keyring for future-proofing:

  ```bash
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
  ```

* Install ROS Melodic and core tools:

  ```bash
  sudo apt-get update
  sudo apt-get install -y \
    ros-melodic-desktop-full \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    ros-melodic-turtlesim ros-melodic-rqt ros-melodic-rqt-common-plugins \
    gazebo9 libgazebo9-dev
  ```

* Install additional ROS tools and controllers:

  ```bash
  sudo apt-get install -y \
    ros-melodic-joint-state-publisher \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-robot-state-publisher \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    ros-melodic-xacro \
    ros-melodic-rviz \
    ros-melodic-controller-manager \
    ros-melodic-transmission-interface \
    ros-melodic-effort-controllers \
    ros-melodic-position-controllers \
    ros-melodic-velocity-controllers
  ```

* Install SLAM and navigation stack:

  ```bash
  sudo apt-get install -y \
    ros-melodic-slam-gmapping \
    ros-melodic-hector-slam \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-amcl \
    ros-melodic-navfn \
    ros-melodic-costmap-2d \
    ros-melodic-global-planner \
    ros-melodic-base-local-planner \
    ros-melodic-teb-local-planner \
    ros-melodic-dwa-local-planner \
    ros-melodic-robot-localization \
    ros-melodic-rqt-robot-steering
  ```

* Install more tools and essentials for development and robot control:

  ```bash
  sudo apt-get install -y \
    ros-melodic-gazebo-ros-control \
    ros-melodic-urdf \
    ros-melodic-urdf-tutorial \
    ros-melodic-xacro \
    ros-melodic-joint-state-publisher \
    ros-melodic-robot-state-publisher \
    ros-melodic-urdfdom-py \
    ros-melodic-rviz \
    ros-melodic-rqt-robot-steering \
    ros-melodic-rqt-graph \
    ros-melodic-rqt-console \
    ros-melodic-rqt-gui \
    ros-melodic-rqt-common-plugins \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-controller-manager \
    ros-melodic-diff-drive-controller \
    ros-melodic-robot-localization \
    ros-melodic-tf \
    ros-melodic-tf2 \
    ros-melodic-roslaunch \
    ros-melodic-turtlesim \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-joint-trajectory-controller \
    python-rospkg \
    python-yaml \
    ros-melodic-joy \
    openssh-server \
    net-tools \
    libuvc-dev \
    libopenni2-dev \
    lsof \
    iputils-ping \
    iproute2 \
    xauth \
    python-catkin-tools \
    build-essential \
    nano vim tmux screen \
    wget curl git lsb-release sudo
  ```

* Install Python dependencies needed for teleop and scripts:

  ```bash
  sudo apt-get install -y python-pip python3-pip
  pip install --no-cache-dir pynput==1.6.3
  python3 -m pip install --no-cache-dir pynput==1.6.3 pyyaml
  ```

* Initialize `rosdep` (if not already done):

  ```bash
  sudo rosdep init
  rosdep update
  rosdep update --rosdistro=melodic
  ```

* (Optional) Configure SSH for remote access (VS Code remote, SSH terminal, etc.):

  ```bash
  sudo mkdir -p /var/run/sshd
  sudo apt-get install -y openssh-server
  sudo service ssh start
  sudo passwd $USER
  ```

* Add automatic workspace sourcing and JetAuto environment variables to your `~/.bashrc`:

  ```bash
  echo "source ~/jetauto_ws/devel/setup.bash" >> ~/.bashrc
  echo 'export LIDAR_TYPE="A1"' >> ~/.bashrc
  echo 'export DEPTH_CAMERA_TYPE="AstraProPlus"' >> ~/.bashrc
  echo 'export MACHINE_TYPE="JetAutoPro"' >> ~/.bashrc
  echo 'export HOST="/"' >> ~/.bashrc
  echo 'export MASTER="/"' >> ~/.bashrc
  ```

* Configure DISPLAY for X11 GUI (Gazebo, RViz, etc.)—set to `:0` if using VcXsrv or similar:

  ```bash
  echo 'export DISPLAY=:0' >> ~/.bashrc
  ```

  Or set to your actual display number as needed.

* Reload your shell to apply all changes:

  ```bash
  source ~/.bashrc
  ```

---

### **20. Update Git Directory and Pull Latest Files**

Navigate to your mounted folder workspace, locate your project git directory for college, and update all files with:

```bash
git pull
```

*Note: This does not include build, devel, logs, and similar files/folders.*

---

### **21. Extract ZIP Workspace Again and Merge**

Extract the JetAuto workspace ZIP file. Overwrite with your files from Git to ensure that the workspace contains `build`, `devel`, `logs`, and all required runtime/generated files.

---

### **22. Convert Windows Line Endings in Python Files**

If you edited any files from Windows, convert all Python files to Unix (LF) line endings:

```bash
find ~/jetauto_ws -type f -name "*.py" -exec dos2unix {} +
```

---

### **23. Make All Python Scripts Executable**

Re-run the command to ensure all Python files in your workspace are executable:

```bash
find ~/jetauto_ws -type f -name "*.py" -exec chmod +x {} +
```

---

### **24. Launch Simulation and Project Node as Before**

**Terminal 1:**

```bash
cd ~/jetauto_ws
source devel/setup.bash
roslaunch jetauto_gazebo worlds.launch
```

**Terminal 2:**

```bash
cd ~/jetauto_ws
source devel/setup.bash
rosrun project2_jetauto_control jetauto_square.py
```

---

### **25. Success: JetAuto Simulation Working on WSL!**

You now have the full JetAuto simulation and Project 2 working on Windows Subsystem for Linux, with all code and configuration synchronized between your systems.
