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
- **A-2:** Virtual Machine on Windows

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
  - If you don't have an NVIDIA GPU, use **Part A-2: Virtual Machine**



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


## Part A-2: Virtual Machine for Windows Users

This section is for students who cannot install a dual-boot system or don't have an NVIDIA GPU.

### Prerequisites

- Connected to **NTHU network** (Wi-Fi is sufficient)
- At least **50GB** of free disk space

---

### Step 1: Download FileZilla Client

Download from: [FileZilla Official Site](https://filezilla-project.org/download.php?platform=win64)

---

### Step 2: Connect to FTP Server

Use the following credentials in FileZilla:

| Field | Value |
|-------|-------|
| **Host** | `140.114.58.210` |
| **Username** | `Deeplearning` |
| **Password** | `111111` |
| **Port** | *Leave blank* |

Then press **Quickconnect**

---

### Step 3: Download the Virtual Machine

1. Navigate to **D415_VM** folder (double left-click)
2. Right-click on `deep_learning...zip`
3. Choose **Download**

💡 **Tip:** Make sure to select the correct local folder to save the .zip file

---

### Step 4: Extract the ZIP File

After extraction, the folder should contain all necessary files.

---

### Step 5: Install VMware Workstation Pro 17.6.1

Download from: [VMware Workstation Pro (Free for Personal Use)](https://blogs.vmware.com/workstation/2024/05/vmware-workstation-pro-now-available-free-for-personal-use.html)

💡 **Alternative:** If you can't download the exe, get it from the FTP server

**Installation:**
- Choose **For Personal Use** during installation

---

### Step 6: Import the Virtual Machine

1. Open **VMware Workstation**
2. Choose **Open a Virtual Machine**
3. Locate the folder where you extracted the .zip file
4. Name your virtual machine
5. Press **Import**

**Note:** The import process takes approximately 5–10 minutes

---

## Part B: Setup D415 Camera and Record Data

In this section, we will turn on the D415 camera and record a bag file.

Refer to the "How to Run" section in the README file for detailed setup instructions:

[hrc-pme/Deep_learning_d415_ROS2](https://github.com/hrc-pme/Deep_learning_d415_ROS2)

---

## ✅ Checkpoints & Grading

### Checkpoint 1: Record Rosbag (80 points)

**Task:** Record a 10-second video using the rosbag command

**Requirements:**
- Use rosbag command to record camera data
- Duration: 10 seconds
- Capture a screenshot of the rosbag information (see example below)

**Reference:** [ROS Rosbag Command Line Guide](https://wiki.ros.org/rosbag/Commandline)

**Example Output:**
```
path:        your_bag_file.bag
version:     2.0
duration:    10.0s
start:       Jan 01 2025 00:00:00.00
end:         Jan 01 2025 00:00:10.00
size:        XXX.X MB
messages:    XXXX
compression: none [X/X chunks]
types:       sensor_msgs/Image
topics:      /camera/color/image_raw   XXX msgs    : sensor_msgs/Image
```

---

### Checkpoint 2: ROS Message Subscriber (20 points)

**Task:** Write a Python or C++ script to subscribe to the color image topic

**Requirements:**
1. Subscribe to the color image topic from the rosbag
2. Use `rospy.loginfo` (Python) or equivalent to display:
   - Image timestamp
   - A notice string

---

## Summary

| Part | Description | Points |
|------|-------------|--------|
| A-1 | Linux/Ubuntu Dual System | +10 (Bonus) |
| A-2 | Virtual Machine Setup | Required (if no GPU) |
| B-1 | Record Rosbag | 80 |
| B-2 | ROS Subscriber Script | 20 |
| **Total** | | **100 + 10 (Bonus)** |

---
