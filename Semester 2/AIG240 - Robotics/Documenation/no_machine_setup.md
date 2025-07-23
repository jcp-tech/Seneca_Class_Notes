# 🚀 NoMachine Client Setup & Usage on WSL/Ubuntu (to connect to JetAuto Robot)

---

## 1️⃣ **Initial One-Time Setup (WSL/Ubuntu Side)**

### **A. Install NoMachine Client**

```bash
# Update package index
sudo apt update

# Download and install the latest NoMachine .deb package (Client + Server included)
wget https://download.nomachine.com/download/9.0/Linux/nomachine_9.0.188_11_amd64.deb
sudo dpkg -i nomachine_9.0.188_11_amd64.deb
sudo apt-get install -f  # Fix any missing dependencies
```

> *You only need to do this once per WSL/Ubuntu installation.*

---

### **B. Ensure GUI Support in WSL**

* **If you’re on Windows 11 with WSLg:**
  No action needed—GUI apps work out-of-the-box.
* **If on Windows 10, or no WSLg:**
  Install and run an X server (like [VcXsrv](https://sourceforge.net/projects/vcxsrv/)) on Windows.

  * Launch VcXsrv before running GUI apps from WSL.

---

## 2️⃣ **Connecting to Your Robot (Each Session)**

### **A. Get the Robot’s IP Address**

On your robot (the device running NoMachine server):

```bash
hostname -I
# or
ip addr
```

> **Note the IP address (e.g., `192.168.1.50`)**

---

### **B. Launch NoMachine Client from WSL**

```bash
/usr/NX/bin/nxplayer
```

* This opens the NoMachine GUI client on your Windows desktop (via WSLg or X server).
* If you get a “command not found” error, ensure the install was successful.

---

### **C. Create a New Connection**

1. In the NoMachine GUI, click **New** or **Add**.
2. **Protocol:** NX
3. **Host:** *(Robot’s IP, e.g.,)* `192.168.1.50`
4. **Port:** `4000` (default)
5. Click **Connect**.
6. **Login:** Use your robot’s username and password.
7. Accept security prompts if it’s the first connection.

---

## 3️⃣ **Tips & Troubleshooting**

* **If you see only a black screen or a terminal:**

  * The robot may not be running a desktop environment.
  * Ensure the robot has something like `xfce4`, `mate`, or `gnome` installed and running.
* **Firewall issues:**

  * Ensure Windows Firewall and any firewall on the robot allow port 4000/TCP.
* **Audio, clipboard, or performance:**

  * Tweak in NoMachine’s connection settings for better speed/quality.

---

## 4️⃣ **Quick Reference: Essential Commands**

| Action                              | Command                                           |
| ----------------------------------- | ------------------------------------------------- |
| **Start NoMachine Client**          | `/usr/NX/bin/nxplayer`                            |
| **Check if NoMachine is installed** | `which nxplayer` or `ls /usr/NX/bin/nxplayer`     |
<!-- | **Remove NoMachine**                | `sudo /usr/NX/scripts/setup/nxserver --uninstall` | -->

---

## 5️⃣ **Summary Table**

| Step | WSL/Ubuntu (Client)        | JetAuto Robot (Server)          |
| ---- | -------------------------- | ------------------------------- |
| 1    | Install NoMachine client   | Install NoMachine server        |
| 2    | Ensure GUI (WSLg/X server) | Ensure desktop env (XFCE, etc.) |
| 3    | Launch `nxplayer`          | Run NoMachine service           |
| 4    | Connect to robot’s IP:4000 | Allow login                     |

---

## ✅ **You’re Done!**

You can now **remotely control your robot’s desktop from your WSL/Ubuntu environment** using NoMachine.

---

### *If you want this as a markdown `.md` file or a printable PDF, just ask!*

If you want a robot-side setup checklist too, let me know.
