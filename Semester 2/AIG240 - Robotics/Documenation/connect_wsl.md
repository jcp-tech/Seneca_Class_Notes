
# 🖥️ Connect to WSL (Ubuntu 18.04) with ROS Melodic + TurtleSim

This guide walks you through installing WSL, setting up Ubuntu 18.04, installing ROS Melodic, and launching GUI-based TurtleSim simulation.

---

## 🧰 Install WSL (Windows Subsystem for Linux)

### Step 1: Check if WSL is Installed

```powershell
wsl --list --online
```

### Step 2: (Optional but Recommended) Install WSL 2

```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
wsl --set-default-version 2
```

> 📝 WSL 2 is faster, supports Docker, and is required for WSLg GUI support.

---

## 🖼️ GUI with WSLg (Windows 11 + WSL 2)

If you're on **Windows 11**, WSLg allows GUI apps to run natively.

### ✅ Steps to enable GUI:
```powershell
wsl --version
wsl --update
```

Launch apps like TurtleSim:
```bash
rosrun turtlesim turtlesim_node
```

> 🐢 GUI should appear automatically—no DISPLAY config needed.

---

## 🔁 Rename Ubuntu 18.04 WSL Distro to `Seneca`

### Step A: Export Existing Instance
```powershell
wsl --export Ubuntu-18.04 C:\WSL\ubuntu18_backup.tar
```

### Step B: Unregister Old Instance
```powershell
wsl --unregister Ubuntu-18.04
```

### Step C: Import with New Name
```powershell
wsl --import Seneca C:\WSL\Seneca C:\WSL\ubuntu18_backup.tar
```

### Step D: Launch WSL
```powershell
wsl -d Seneca
```

---

## 🐧 Install Ubuntu 18.04

```powershell
wsl --install -d Ubuntu-18.04
```

If there's an error:
```powershell
wsl --list --online
wsl --install -d <ExactName>
```

---

## 👤 Initial Setup

- Set username: `jetauto`
- Create secure password when prompted

---

## 🔧 Install ROS Melodic in Ubuntu 18.04 (WSL)

### Step 1: Update System

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release wget build-essential -y
```

### Step 2: Add ROS Sources

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

### Step 3: Install ROS

```bash
sudo apt install ros-melodic-desktop-full -y
```

### Step 4: Source ROS

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Install ROS Tools

```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
```

### Step 6: Initialize rosdep

```bash
sudo rosdep init || true
rosdep update --rosdistro=melodic
```

---

## 🖼️ Enable GUI with VcXsrv (if not using WSLg)

1. Download VcXsrv: https://sourceforge.net/projects/vcxsrv/
2. Launch with:
   - Multiple Windows
   - Display number: 0
   - Start no client
   - Disable access control

3. Configure DISPLAY in WSL:
```bash
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
source ~/.bashrc
```

---

✅ ROS + TurtleSim now working in WSL!
