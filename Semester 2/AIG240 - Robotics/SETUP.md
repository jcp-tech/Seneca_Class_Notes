# 🧰 Install WSL (Windows Subsystem for Linux)

### Step 1: Check if WSL is Installed

```powershell
wsl --list --online
```

### Step 3: \[Optional] Install WSL 2 (if supported)

```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
wsl --set-default-version 2
```

> 📝 WSL 2 is faster and supports Docker, and required for WSLg.

---

# 🖼️ GUI with WSLg (Windows 11 + WSL 2)

If you're on **Windows 11** and using **WSL 2**, you can use **WSLg**, which has **built-in GUI support** with no configuration needed.

### ✅ To enable GUI with WSLg:

1. Ensure you're using **Windows 11** and WSL version 2:

```powershell
wsl --version
```

2. Update WSL:

```powershell
wsl --update
```

3. Launch GUI apps directly:

```bash
rosrun turtlesim turtlesim_node
```

> 🐢 The turtle window will open without needing VcXsrv or setting the DISPLAY variable.

---

# 🔁 Rename Ubuntu 18.04 WSL to "Seneca"

### Step A: Export the existing Ubuntu 18.04 instance

```powershell
wsl --export Ubuntu-18.04 C:\WSL\ubuntu18_backup.tar
```

### Step B: Unregister the old Ubuntu-18.04 instance (optional but recommended)

```powershell
wsl --unregister Ubuntu-18.04
```

### Step C: Import the backup under the new name "Seneca"

```powershell
wsl --import Seneca C:\WSL\Seneca C:\WSL\ubuntu18_backup.tar
```

### Step D: Launch it anytime with:

```powershell
wsl -d Seneca
```

---

# 🐧 Install Ubuntu 18.04 with WSL

### Step 4: Install Ubuntu 18.04

```powershell
wsl --install -d Ubuntu-18.04
```

> 💡 If you get an error, try:

```powershell
wsl --list --online
wsl --install -d <ExactNameForUbuntu-18.04>
```

### Step 5: Launch Ubuntu 18.04

* It will prompt for a new UNIX username.
* Enter:

```bash
jetauto
```

* Choose a secure password when prompted.

---

# 🔧 Setup ROS Melodic in Ubuntu 18.04 (WSL)

### Step 6: Update & Install essentials

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release wget build-essential -y
```

### Step 7: Add ROS repository

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```bash
sudo apt update
```

### Step 8: Install ROS Melodic Desktop-Full

```bash
sudo apt install ros-melodic-desktop-full -y
```

### Step 9: Source ROS automatically

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 10: Install ROS Tools

```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
```

### Step 11: Initialize rosdep

```bash
jetauto@MSS-CAD-LT-001:~$ sudo rosdep init
ERROR: default sources list file already exists:
        /etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize
rosdep update --rosdistro=melodic
```

---

# 🖼️ Enable GUI (TurtleSim) with VcXsrv (if not using WSLg)

> ❗ Skip this section if you're using Windows 11 + WSL 2 with WSLg.

### Step 12: Install & Launch VcXsrv on Windows

* Download from: [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/)
* Launch with:

  * Multiple Windows
  * Display Number: 0
  * Start no client
  * Disable access control ✅

### Step 13: Configure DISPLAY in Ubuntu

```bash
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
source ~/.bashrc
```

---

# 🐢 Test TurtleSim

\*For full interactive instructions, see the original lab guide at: \**[Seneca AIG240 Lab 1](https://seneca-bsa.github.io/bsa/aig240/lab1/)*

---

### Using rosrun

The command `rosrun` allows you to use the package name to directly run a node within a package (without having to know the package path). Usage:

```bash
rosrun [package_name] [node_name]
```

So now we can run the `turtlesim_node` in the `turtlesim` package. In a new terminal:

```bash
rosrun turtlesim turtlesim_node
```

The simulator window should appear, with a random turtle in the center.

Then open a **new terminal** to control the turtle:

```bash
rosrun turtlesim turtle_teleop_key
```

Make sure this terminal is focused so it receives arrow key input.

> ✅ Use the arrow keys on your keyboard to move the turtle around the screen. It will draw a line as it moves using its "pen."

---

### Using rqt

`rqt` is a GUI tool for interacting with ROS topics and services.

1. Open a new terminal:

```bash
rqt
```

2. When the window opens, go to:

   * `Plugins > Services > Service Caller`
3. Refresh the service list.
4. Choose the `/spawn` service and input:

   * `name`: `turtle2`
   * `x`: `1.0`
   * `y`: `1.0`
5. Click **Call** to spawn a second turtle.

To change the pen color of turtle1:

* Choose `/turtle1/set_pen`
* Set `r = 255`, `width = 5`
* Click **Call** again

Move turtle1 again in your teleop terminal to see the red line appear.

---

### Remapping Teleop Control to turtle2

To control turtle2, run another teleop node with a remapped topic:

```bash
rosrun turtlesim turtle_teleop_key turtle1/cmd_vel:=turtle2/cmd_vel
```

Now:

* Use your original teleop terminal to move turtle1
* Use this new terminal to move turtle2

---

### Lab Challenge

* Create a third turtle using `/spawn`
* Set its pen color to **green** (`g = 255`)
* Control it using a remapped teleop node

---

### Terminal 1:

```bash
roscore
```

> 🧠 Starts the ROS master node.

### Terminal 2:

```bash
rosrun turtlesim turtlesim_node
```

> 🐢 Launches the GUI simulator with the default turtle.

### Terminal 3:

```bash
rosrun turtlesim turtle_teleop_key __name:=teleop_turtle1
```

> 🎮 Controls turtle1 using arrow keys (this terminal must be focused).
>
> The `__name:=teleop_turtle1` isn't Necessary Unless you are Using Multiple Turtles.

### Terminal 4 (optional):

```bash
rosrun turtlesim turtle_teleop_key __name:=teleop_turtle2 turtle1/cmd_vel:=turtle2/cmd_vel
```

> 🎮 Controls turtle2 using a remapped teleop node and a unique node name.

### Terminal 5 (optional):

```bash
rqt
```

> 🧰 Launches the GUI tool to spawn turtles, set pen color, etc.

---

### 🛠️ Troubleshooting Tips

* 🛑 **Avoid duplicate node names:** Always give each `turtle_teleop_key` instance a unique node name using `__name:=...`.
* ✅ **Use valid turtle names only** (e.g., `turtle2`, `turtle3`). Avoid symbols like `!`, `@`, etc.
* 🐢 To spawn via terminal (instead of rqt):

```bash
rosservice call /spawn 1.0 1.0 0.0 "turtle2"
```

* 🎯 To control turtle2 from the terminal:

```bash
rosrun turtlesim turtle_teleop_key cmd_vel:=/turtle2/cmd_vel
```

Make sure `turtlesim_node` is running before calling `/spawn` or `/set_pen`.

***NOTE:*** *There might be a Problem (hopefully because of ************WSL************ that you can't control both Turtles but this is to be Looked into.*

---

### *💻* Your Terminal Setup Should Look Like:

| Terminal       | Command                              |
| -------------- | ------------------------------------ |
| 🖥️ Terminal 1 | `roscore`                            |
| 🐢 Terminal 2  | `rosrun turtlesim turtlesim_node`    |
| 🎮 Terminal 3  | `rosrun turtlesim turtle_teleop_key` |

> 📝 Make sure Terminal 3 is **focused and active** when pressing arrow keys.

```

> ✅ Use arrow keys to move the turtle.

---

✅ You now have ROS Melodic + TurtleSim working inside WSL with GUI support!

```