---
title: "Development Environment Setup Guide"
sidebar_label: "Setup Guide"
sidebar_position: 2
description: "Step-by-step instructions for installing Ubuntu, ROS 2 Humble, Python, and VS Code for Physical AI development"
keywords: [setup, installation, ubuntu, ros2, humble, python, vscode, development environment]
---

# Development Environment Setup Guide

This guide walks you through setting up a complete development environment for Physical AI and ROS 2 programming. Follow these steps in order to ensure all dependencies are correctly installed and configured.

---

## Prerequisites

Before beginning, ensure you have:
- A computer with at least 8GB RAM (16GB recommended)
- 50GB free disk space
- Stable internet connection for downloading packages
- Administrator/root access to install software

---

## 1. Ubuntu 22.04 Installation

ROS 2 Humble officially supports Ubuntu 22.04 LTS (Jammy Jellyfish). You can install Ubuntu either as your primary OS, in a dual-boot configuration, or using a virtual machine.

### Option A: Native Installation

1. **Download Ubuntu 22.04 LTS**
   - Visit [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)
   - Download Ubuntu 22.04.3 LTS (or latest 22.04 version)
   - Create a bootable USB drive using [Rufus](https://rufus.ie/) (Windows) or [Etcher](https://www.balena.io/etcher/) (cross-platform)

2. **Install Ubuntu**
   - Boot from USB drive
   - Follow installation wizard
   - Choose "Install Ubuntu" (not "Try Ubuntu")
   - Select "Normal installation" with updates and third-party software
   - Configure disk partitioning (minimum 40GB for root partition)
   - Create user account and set password

3. **Verify Installation**
   ```bash
   lsb_release -a
   ```
   Expected output:
   ```
   Distributor ID: Ubuntu
   Description:    Ubuntu 22.04.3 LTS
   Release:        22.04
   Codename:       jammy
   ```

### Option B: Virtual Machine

If you prefer to use a virtual machine:

1. **Install VirtualBox or VMware**
   - Download [VirtualBox](https://www.virtualbox.org/) (free) or VMware Workstation Player
   - Install following default settings

2. **Create Ubuntu VM**
   - Allocate at least 4GB RAM (8GB recommended)
   - Create 50GB virtual hard disk (dynamically allocated)
   - Attach Ubuntu 22.04 ISO as CD/DVD
   - Boot VM and follow installation steps from Option A

3. **Install Guest Additions** (VirtualBox only)
   ```bash
   sudo apt update
   sudo apt install build-essential dkms linux-headers-$(uname -r)
   # Insert Guest Additions CD from VirtualBox menu
   sudo sh /media/$USER/VBox*/VBoxLinuxAdditions.run
   sudo reboot
   ```

### Update System

After installation, update all packages:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential curl git wget vim nano
```

---

## 2. ROS 2 Humble Installation

ROS 2 Humble Hawksbill is the LTS (Long-Term Support) release for Ubuntu 22.04.

### Set Locale

Ensure UTF-8 locale is set:

```bash
locale  # Check current settings
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Enable Ubuntu Universe Repository

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

### Add ROS 2 APT Repository

```bash
# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Packages

```bash
# Update package index
sudo apt update

# Install ROS 2 Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-humble-desktop

# Install development tools and ROS tools
sudo apt install -y ros-dev-tools
```

Installation will take 10-20 minutes depending on internet speed.

### Verify ROS 2 Installation

```bash
# Source ROS 2 setup file
source /opt/ros/humble/setup.bash

# Check ROS 2 version
ros2 --version
```

Expected output: `ros2 cli version 0.18.x` or similar

### Test ROS 2 with Talker/Listener Demo

**Terminal 1** (Publisher):
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2** (Subscriber):
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see the listener receiving messages published by the talker.

### Configure Environment (Automatic Sourcing)

Add ROS 2 setup to your `.bashrc` to automatically source it in new terminals:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Python Environment Setup

ROS 2 Humble uses Python 3.10, which is the default Python 3 version in Ubuntu 22.04.

### Verify Python Version

```bash
python3 --version
```

Expected output: `Python 3.10.x`

### Install Python Development Tools

```bash
sudo apt install -y python3-pip python3-venv python3-dev
```

### Install Essential Python Packages

```bash
# Install colcon (ROS 2 build tool)
sudo apt install -y python3-colcon-common-extensions

# Install rosdep (dependency management)
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

# Install argcomplete (tab completion)
sudo apt install -y python3-argcomplete
```

### Create Python Virtual Environment (Optional)

For project-specific dependencies, create a virtual environment:

```bash
mkdir -p ~/robot_ws/python_envs
cd ~/robot_ws/python_envs
python3 -m venv ros2_env

# Activate virtual environment
source ros2_env/bin/activate

# Install common packages
pip install numpy matplotlib scipy
```

**Note**: When using virtual environments with ROS 2, ensure you source both the ROS 2 setup and activate the virtual environment.

---

## 4. ROS 2 Workspace Setup

Create a workspace for your ROS 2 projects:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace (initially empty)
colcon build

# Source workspace overlay
source ~/ros2_ws/install/setup.bash

# Add workspace overlay to .bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Verify Workspace

```bash
# Check ROS environment variables
printenv | grep ROS
```

You should see variables like `ROS_DISTRO=humble` and `ROS_VERSION=2`.

---

## 5. VS Code Installation and Configuration

Visual Studio Code is the recommended IDE for ROS 2 development.

### Install VS Code

**Option A: Via APT Repository (Recommended)**

```bash
# Import Microsoft GPG key
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg

# Add VS Code repository
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

# Install VS Code
sudo apt update
sudo apt install -y code
```

**Option B: Download .deb Package**

```bash
# Download latest .deb package
wget -O ~/Downloads/code_latest_amd64.deb "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64"

# Install package
sudo dpkg -i ~/Downloads/code_latest_amd64.deb
sudo apt install -f  # Fix dependencies if needed
```

### Install Essential VS Code Extensions

Launch VS Code and install these extensions:

1. **Python** (ms-python.python)
   - Syntax highlighting, IntelliSense, debugging

2. **ROS** (ms-iot.vscode-ros)
   - ROS/ROS 2 support, package creation, launch file debugging

3. **C/C++** (ms-vscode.cpptools)
   - Required for C++ ROS 2 nodes (if you work with C++)

4. **CMake Tools** (ms-vscode.cmake-tools)
   - CMake support for ROS 2 packages

5. **XML Tools** (redhat.vscode-xml)
   - XML formatting for package.xml and launch files

Install via command line:

```bash
code --install-extension ms-python.python
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension redhat.vscode-xml
```

### Configure VS Code Settings for ROS 2

Create workspace settings file:

```bash
mkdir -p ~/ros2_ws/.vscode
nano ~/ros2_ws/.vscode/settings.json
```

Add the following configuration:

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ],
  "cmake.configureOnOpen": false,
  "files.associations": {
    "*.launch": "xml",
    "*.xacro": "xml",
    "*.urdf": "xml"
  },
  "ros.distro": "humble"
}
```

Save and close the file.

### Open Workspace in VS Code

```bash
cd ~/ros2_ws
code .
```

---

## 6. Testing Your Setup

### Create a Test Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python test_package --dependencies rclpy std_msgs
```

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select test_package
source install/setup.bash
```

### Create a Simple Publisher Node

Create file `~/ros2_ws/src/test_package/test_package/simple_pub.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make it executable:

```bash
chmod +x ~/ros2_ws/src/test_package/test_package/simple_pub.py
```

Update `~/ros2_ws/src/test_package/setup.py` to include the entry point:

```python
entry_points={
    'console_scripts': [
        'simple_pub = test_package.simple_pub:main',
    ],
},
```

Rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select test_package
source install/setup.bash
```

Run the node:

```bash
ros2 run test_package simple_pub
```

You should see messages being published every second.

---

## 7. Troubleshooting Common Issues

### Issue: "ros2: command not found"

**Solution**: Source the ROS 2 setup file
```bash
source /opt/ros/humble/setup.bash
```

Make it permanent by adding to `.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Issue: "colcon: command not found"

**Solution**: Install colcon
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

---

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Solution**: Ensure ROS 2 environment is sourced
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Check Python path includes ROS 2 packages:
```bash
python3 -c "import sys; print('\n'.join(sys.path))"
```

---

### Issue: VS Code Python IntelliSense not finding ROS 2 modules

**Solution**: Update VS Code settings.json with correct Python paths (see section 5 above)

Alternatively, in VS Code:
1. Press `Ctrl+Shift+P`
2. Type "Python: Select Interpreter"
3. Choose `/usr/bin/python3` (system Python with ROS 2 packages)

---

### Issue: "Network is unreachable" or DDS discovery issues

**Solution**: Configure DDS middleware
```bash
export ROS_DOMAIN_ID=0  # Use domain 0-101
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Or rmw_fastrtps_cpp
```

Add to `.bashrc` for persistence:
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

---

### Issue: Permission denied when accessing USB devices (e.g., Arduino, cameras)

**Solution**: Add user to dialout group
```bash
sudo usermod -aG dialout $USER
sudo reboot
```

---

### Issue: Build fails with "CMake Error"

**Solution**: Clean workspace and rebuild
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --cmake-clean-cache
```

---

## 8. Additional Tools (Optional)

### RViz (3D Visualization)

Already included with `ros-humble-desktop`. Launch with:
```bash
rviz2
```

### Gazebo (Robot Simulation)

Install Gazebo Classic:
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

Or install Gazebo Fortress (newer):
```bash
sudo apt install -y ros-humble-ros-gz
```

### rqt (ROS 2 GUI Tools)

```bash
sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins
rqt  # Launch GUI
```

### Documentation Tools

```bash
sudo apt install -y ros-humble-rosdoc2
```

---

## 9. Verification Checklist

Before proceeding with the textbook exercises, verify all components are working:

- [ ] Ubuntu 22.04 installed and updated
- [ ] ROS 2 Humble installed (`ros2 --version` works)
- [ ] Python 3.10+ installed (`python3 --version`)
- [ ] ROS 2 workspace created and buildable (`colcon build` succeeds)
- [ ] VS Code installed with ROS extensions
- [ ] Test package builds and runs successfully
- [ ] ROS 2 environment automatically sources on new terminal

---

## Next Steps

With your environment fully configured, you're ready to begin learning Physical AI and ROS 2 development:

1. Review [Chapter 1: Introduction to Physical AI](../week-01-02-intro-physical-ai/index.md)
2. Complete [Chapter 3: ROS 2 Part 1 exercises](../week-03-ros2-part1/exercises.md)
3. Explore the [Glossary](./glossary.md) for terminology reference

---

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Ubuntu Community Help](https://help.ubuntu.com/)
- [VS Code Documentation](https://code.visualstudio.com/docs)
- [Python Documentation](https://docs.python.org/3/)

---

**Last Updated**: 2025-11-30
**ROS 2 Version**: Humble Hawksbill
**Ubuntu Version**: 22.04 LTS (Jammy Jellyfish)
