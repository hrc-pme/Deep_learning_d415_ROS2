# Lab 1: Accessing the Camera via ROS2

**Author:** Lester, Ching-I

---

## Overview

In this lab, we will use ROS2 to activate a depth camera. You are encouraged to use Linux/Ubuntu for further applications, especially in robotics. However, a virtual machine will also be provided for those using Windows. 

### Learning Objectives
- Activate a depth camera using ROS2
- Record a rosbag for logging data
- Set up development environments for robotics applications

---

## Tutorial Structure

The tutorial includes two main parts:

### Part A: Environment Setup
- **A-1:** Linux/Ubuntu Dual System **(Bonus +10 points)**
- **A-2:** WSL2 on Windows

### Part B: Camera Setup and Data Recording
- Set up the D415 camera and record data

---

## Part A-1: Linux/Ubuntu Dual System

> **Recommended but Optional | Bonus: +10 points**

Installation issues may vary by computer and involve deeper system access, which carries some risk but will expand your engineering knowledge. You are encouraged to complete the installation independently.

### Support Policy
- If you need help, please email our TAs
- While TAs will assist as much as possible, not all issues can be guaranteed to be resolved
- TAs may adjust grades based on the level of assistance required, as we aim to foster independent problem-solving

---

### ⚠️ Important Reminders

**Before starting, please ensure:**

1. **Windows BitLocker is disabled**
   - [More information here](https://www.asus.com/support/faq/1044341/)

2. **BIOS Settings:**
   - Login to your BIOS
   - Disable **Fast Boot**
   - Disable **Security Boot** (set an admin password first if locked)

3. **Wi-Fi Card Compatibility:**
   - Press `Ctrl + Shift + Esc` in Windows to open Task Manager
   - Check your Wi-Fi card name
   - ⚠️ If not using Intel Wi-Fi card, you may need to share network via phone/cable

4. **Black Screen Issues:**
   - If you encounter a black screen during installation, choose **Safe Graphics** mode when using the USB boot device

5. **CRITICAL WARNING:**
   - **DO NOT choose any "Erase" option during installation!**
   - This will empty your Windows partition (**IRREVERSIBLE**)

---

### Prerequisites

**Required:**
- Bootable USB drive (minimum 16GB)
  - Ubuntu 22.04 recommended (especially for Intel 13th/14th gen CPUs)
  - Any Ubuntu version is acceptable
- NVIDIA GPU (GTX or RTX series)
  - If you don't have an NVIDIA GPU, use **Part A-2: WSL2**

**Pre-Installation Checklist:**
- BitLocker is disabled (if applicable)
- Wi-Fi card compatibility verified
- BIOS settings configured
- At least 128GB free space for Ubuntu (⅓ of storage recommended)



---

### Installation Steps

#### Step 1: Install Ubuntu

Follow this video tutorial for dual system installation:
- [Ubuntu Dual Boot Installation Guide](https://www.youtube.com/watch?v=tEh1RfmbTBY)

> **NOTE**: At least 128GB free space for Ubuntu (⅓ of storage recommended)
---

#### Step 2: Install Required Utilities

Open Terminal with `Ctrl + Alt + T`

```bash
# Update system and install basic tools
sudo apt update
sudo apt install terminator curl git vim net-tools
```

---

#### Step 3: Install NVIDIA Graphics Driver

```bash
# Update package list
sudo apt update

# List available NVIDIA drivers
sudo ubuntu-drivers list

# Install driver (replace 5xx with your version: 525, 535, 555, etc.)
sudo apt install nvidia-driver-5xx

# Reboot system
sudo reboot now
```

**Troubleshooting:**
- If your computer freezes at the logo:
  1. Choose **Ubuntu Advanced Options**
  2. Select another kernel version (without Safe Mode)

**Verify Installation:**
```bash
# Check if GPU is working correctly
nvidia-smi
```

---

#### Step 4: Install Docker

[Official Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

```bash
# Download and run Docker installation script
curl -fsSL https://test.docker.com -o test-docker.sh
sudo sh test-docker.sh

# Fix daemon permission issues
sudo groupadd docker
sudo usermod -aG docker $USER

# Reboot to apply changes
sudo reboot now
```

---

#### Step 5: Install NVIDIA Docker Toolkit

[Official NVIDIA Container Toolkit Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

```bash
# Add NVIDIA container toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

---

#### Step 6: Setup Git SSH Key

[SSH Key Setup Guide (Chinese)](https://ithelp.ithome.com.tw/articles/10205988)

> **Note:** You need a GitHub account first

**Verify Setup:**
```bash
# Test SSH connection to GitHub
ssh -T git@github.com
```

---

### Next Steps

For Part B tutorial (D415 camera setup), check:
- 🔗 [Deep Learning D415 ROS2 Repository](https://github.com/hrc-pme/Deep_learning_d415_ROS2)

---

## Part A-2: WSL2 on Windows

If you do not have enough storage space for a dual-boot system, you can use **WSL 2 (Windows Subsystem for Linux)** on Windows. This section will guide you through installing WSL 2, Ubuntu, Docker Desktop, and configuring them so that the ROS 2 camera environment can run successfully.

---

### Prerequisites

- Windows 10 version 2004 or higher, or Windows 11
- At least **50GB** of free disk space
- Administrator access to your Windows system

---

### Step 1: Install WSL 2

#### 1.1 Open PowerShell as Administrator

Right-click on the Start button and select **Windows PowerShell (Admin)** or **Terminal (Admin)**

#### 1.2 Install WSL and Ubuntu

Run the following command:

```powershell
wsl --install
```

This will:
- Enable WSL feature
- Install the default Ubuntu distribution
- Set WSL 2 as the default version

#### 1.3 Verify WSL Version

```powershell
wsl --version
```

Make sure your version is **at least 2.0**

#### 1.4 Restart Your Computer

When prompted, restart your computer to complete the installation.

---

### Step 2: Install Ubuntu for WSL

#### 2.1 Install Ubuntu from Microsoft Store

1. Open the **Microsoft Store**
2. Search for **Ubuntu 22.04 LTS** (or just "Ubuntu")
3. Click **Install**



#### 2.2 Launch Ubuntu

**Option 1:** Open from Start Menu

**Option 2:** From PowerShell, type:
```powershell
ubuntu
```

#### 2.3 Initial Setup

When Ubuntu launches for the first time:
1. Create a **username** (e.g., `deeplearning`)
2. Create a **password**

> ⚠️ **Note:** The password will not be visible as you type (this is normal for Linux)

#### 2.4 Update the System

```bash
sudo apt update && sudo apt upgrade -y
```

#### 2.5 Opening WSL in the Future

**Method 1:** Type `Ubuntu` in Windows search bar

**Method 2:** Open PowerShell and type:
```powershell
ubuntu
```

#### 2.6 Verify Installation

Your terminal should look like a standard Ubuntu shell:

```bash
username@DESKTOP:~$
```

**Open VS Code in WSL:**
```bash
code .
```

---

### Step 3: Install Docker Desktop

#### 3.1 Download Docker Desktop

Download from: [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)

**System Architecture:**
- Most users should select **AMD64 (x64)**
- To verify: Open **Device Manager** → **System Information**
- If it shows "x64-based processor", select **AMD64**

#### 3.2 Install Docker Desktop

1. Run the installer
2. Follow the installation wizard
3. Accept the default settings

> **Important:** Docker Desktop must remain open while using Docker commands

#### 3.3 Configure WSL Integration

1. Open **Docker Desktop**
2. Go to **Settings** (gear icon)
3. Navigate to **Resources** → **WSL Integration**
4. Enable **"Enable integration with my default WSL distro"**
5. Ensure **Ubuntu** is selected and toggled ON
6. Click **Apply & Restart**


#### 3.4 Verify Docker Installation

Open Ubuntu terminal and run:

```bash
docker --version
docker run hello-world
```

You should see Docker version information and a "Hello from Docker!" message.

---

### Step 4: Connect Camera to WSL

This step allows your WSL environment to access USB devices like the Intel RealSense D415 camera.

#### 4.1 Install USBIPD on Windows

Open **PowerShell as Administrator** and run:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

Or download from: [USBIPD-WIN Releases](https://github.com/dorssel/usbipd-win/releases)

#### 4.2 List Connected USB Devices

In PowerShell (Administrator):

```powershell
usbipd list
```

Find your camera in the list and note its **BUSID** (e.g., `1-25`)

**Example Output:**
```
BUSID  VID:PID    DEVICE                                            STATE
1-25   8086:0b07  Intel(R) RealSense(TM) Depth Camera 415           Not attached
```

#### 4.3 Bind the Camera

Replace `1-25` with your actual BUSID:

```powershell
usbipd bind --busid 1-25
```

Verify the device is now **Shared**:

```powershell
usbipd list
```

**Expected Output:**
```
BUSID  VID:PID    DEVICE                                            STATE
1-25   8086:0b07  Intel(R) RealSense(TM) Depth Camera 415           Shared
```

#### 4.4 Attach Camera to WSL

```powershell
usbipd attach --wsl --busid 1-25 --auto-attach
```

> 💡 **Tip:** The `--auto-attach` flag will automatically reconnect the camera to WSL after reboots

#### 4.5 Verify Camera in WSL

Open Ubuntu terminal and run:

```bash
lsusb
```

You should see an entry like:

```
Bus 001 Device 002: ID 8086:0b07 Intel Corp. RealSense D415
```

---

### Daily Usage Tips

**Every time you want to use the camera:**

1. Open **Docker Desktop** (must be running)
2. If camera is not detected in WSL, reattach it:
   ```powershell
   # In PowerShell (Administrator)
   usbipd attach --wsl --busid 1-25
   ```
3. Open Ubuntu terminal:
   ```powershell
   ubuntu
   ```
4. Verify camera connection:
   ```bash
   lsusb
   ```

---

## Part B: Setup D415 Camera and Record Data

In this section, we will turn on the D415 camera and record a bag file.

### For Linux Users (Part A-1)

#### Open Terminal

Press `Ctrl + Alt + T` to open terminal

Or use VS Code: Press `Ctrl + ~` to open integrated terminal

---

### For WSL2 Users (Part A-2)

#### 1. Ensure Camera is Connected

In **PowerShell (Administrator)**:

```powershell
# Check if camera is attached
usbipd list

# If not attached, attach it
usbipd attach --wsl --busid 1-25
```

#### 2. Open Ubuntu Terminal

**Option 1:** Type `Ubuntu` in Windows search

**Option 2:** In PowerShell:
```powershell
ubuntu
```

#### 3. Open VS Code (Optional but Recommended)

```bash
code .
```

Then press `Ctrl + ~` to open integrated terminal in VS Code

#### 4. Verify Camera in WSL

```bash
lsusb
```

Make sure you see the Intel RealSense camera listed.

---

### Camera Setup and Data Recording

Follow the **"Setup Instructions"** section in the main README:

- [Deep Learning D415 ROS2 - Setup Instructions](https://github.com/hrc-pme/Deep_learning_d415_ROS2)

**Quick Reference:**

1. Navigate to the repository folder
2. Launch the Docker container
3. Start the camera node
4. Record rosbag data

---

## Checkpoints & Grading

### Checkpoint 1: Record Rosbag (80 points)

**Task:** Record a 10-second video using the rosbag command

**Requirements:**
- Use `ros2 bag record` command to record camera data
- Duration: **10 seconds**
- Capture a screenshot of the rosbag information

**Reference:** [ROS 2 Rosbag Recording Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

**Example Commands:**

```bash
# Record all topics for 10 seconds
ros2 bag record -a --duration 10

# Or record specific camera topics
# Or there are some pkg in the repo
```

**Check Bag Info:**

```bash
ros2 bag info <bag_folder_name>
```

**Example Output:**
```
Files:             your_bag_file_0.db3
Bag size:          XXX.X MiB
Storage id:        sqlite3
Duration:          10.0s
Start:             Jan 01 2025 00:00:00.00
End:               Jan 01 2025 00:00:10.00
Messages:          XXXX
Topic information: 
  Topic: /camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: XXX | Serialization Format: cdr
```

---

### Checkpoint 2: ROS Message Subscriber (20 points)

**Task:** Write a Python or C++ script to subscribe to the color image topic

**Requirements:**
1. Subscribe to the color image topic from the rosbag
2. Display using ROS logging:
   - Image timestamp
   - A notice string


---

## Support & Resources

### GitHub Repository
- [Deep Learning D415 ROS2](https://github.com/hrc-pme/Deep_learning_d415_ROS2)

### Additional Resources
- [WSL 2 Installation Guide](https://learn.microsoft.com/en-us/windows/wsl/install)
- [USBIPD-WIN Documentation](https://github.com/dorssel/usbipd-win)
- [Docker Desktop WSL 2 Backend](https://docs.docker.com/desktop/wsl/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

### Contact
- Email TAs for assistance

---

## Summary

| Part | Description | Points |
|------|-------------|--------|
| A-1 | Linux/Ubuntu Dual System | +10 (Bonus) |
| A-2 | WSL2 Setup | Required (alternative to A-1) |
| B-1 | Record Rosbag | 80 |
| B-2 | ROS Subscriber Script | 20 |
| **Total** | | **100 + 10 (Bonus)** |

---

**Good luck!**
