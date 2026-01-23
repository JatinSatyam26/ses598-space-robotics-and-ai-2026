# Quick Start Guide - SES598 Space Robotics and AI

This guide provides fast-track setup instructions for students. For detailed information, see individual assignment READMEs and the [README_IMPROVEMENTS.md](assignments/README_IMPROVEMENTS.md).

---

## 🚀 15-Minute Quick Setup

### Prerequisites Checker

Run this script to check your system before starting:

```bash
#!/bin/bash
echo "=== SES598 System Check ==="
echo ""

# Check OS
echo "OS: $(lsb_release -d | cut -f2)"

# Check ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble found"
    export ROS_DISTRO=humble
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "✓ ROS2 Jazzy found"
    export ROS_DISTRO=jazzy
else
    echo "✗ ROS2 not found - install ROS2 first"
    exit 1
fi

# Check Python
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "Python: $PYTHON_VERSION"

# Check Gazebo
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -n1)
    echo "✓ Gazebo: $GZ_VERSION"
else
    echo "⚠ Gazebo not found"
fi

# Check disk space
DISK_FREE=$(df -h ~ | tail -n1 | awk '{print $4}')
echo "Free disk space: $DISK_FREE"

# Check RAM
RAM_TOTAL=$(free -h | grep Mem | awk '{print $2}')
echo "Total RAM: $RAM_TOTAL"

echo ""
echo "=== Ready to proceed! ==="
```

Save as `check_system.sh`, make executable (`chmod +x check_system.sh`), and run.

---

## Assignment 1: Boustrophedon Navigator (Easiest)

**Time to setup: ~10 minutes**

```bash
# 1. Install dependencies
sudo apt install ros-$ROS_DISTRO-turtlesim ros-$ROS_DISTRO-rqt*
pip3 install numpy matplotlib

# 2. Setup package
cd ~/ros2_ws/src
ln -s ~/ses598-space-robotics-and-ai-2026/assignments/first_order_boustrophedon_navigator .
cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator --symlink-install
source install/setup.bash

# 3. Run
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py

# 4. Tune parameters (in another terminal)
ros2 run rqt_reconfigure rqt_reconfigure
```

**Success check:** You should see a turtle drawing a lawnmower pattern.

---

## Assignment 2: Cart-Pole Control (Medium)

**Time to setup: ~30 minutes**

### Quick Install

```bash
# 1. Install Gazebo (if not installed)
# For Ubuntu 22.04:
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install -y gz-garden

# For Ubuntu 24.04:
# sudo apt install -y gz-harmonic

# 2. Install ROS2-Gazebo bridge
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2

# 3. Install Python dependencies
pip3 install numpy scipy python-control matplotlib

# 4. Setup package
cd ~/ros2_ws/src
ln -s ~/ses598-space-robotics-and-ai-2026/assignments/cart_pole_optimal_control .
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install
source install/setup.bash

# 5. Run
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

**Success check:** Gazebo and RViz open, cart-pole system is visible and cart stays within ±2.5m while pole remains upright despite earthquake forces.

### For Extra Credit (DQN)

```bash
pip3 install torch gymnasium
```

---

## Assignment 3: Drone Control (Most Complex)

**Time to setup: ~1-2 hours** (mostly compilation time)

### One-Shot Installation Script

Save this as `setup_assignment3.sh`:

```bash
#!/bin/bash
set -e

echo "=== Setting up Assignment 3: Drone Control ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS_DISTRO not set${NC}"
    echo "Run: export ROS_DISTRO=humble (or jazzy)"
    exit 1
fi

echo -e "${GREEN}Using ROS_DISTRO: $ROS_DISTRO${NC}"

# Step 1: Install ROS2 dependencies
echo -e "${YELLOW}[1/7] Installing ROS2 dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-px4-msgs \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-cv-bridge \
    python3-opencv

# Step 2: Install RTAB-Map
echo -e "${YELLOW}[2/7] Installing RTAB-Map...${NC}"
sudo apt install -y \
    ros-$ROS_DISTRO-rtabmap \
    ros-$ROS_DISTRO-rtabmap-ros

# Step 3: Install Python dependencies
echo -e "${YELLOW}[3/7] Installing Python dependencies...${NC}"
pip3 install numpy opencv-python transforms3d scipy

# Step 4: Clone and build PX4-Autopilot
echo -e "${YELLOW}[4/7] Setting up PX4-Autopilot (this will take 10-20 minutes)...${NC}"
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    cd ~/
    git clone https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    git checkout 9ac03f03eb
    bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
else
    echo "PX4-Autopilot already exists, skipping clone"
fi

# Step 5: Build PX4
echo -e "${YELLOW}[5/7] Building PX4...${NC}"
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Step 6: Setup ROS2 package
echo -e "${YELLOW}[6/7] Setting up ROS2 package...${NC}"
cd ~/ros2_ws/src
if [ ! -L "terrain_mapping_drone_control" ]; then
    ln -s ~/ses598-space-robotics-and-ai-2026/assignments/terrain_mapping_drone_control .
fi

# Deploy PX4 models
cd ~/ros2_ws/src/terrain_mapping_drone_control
chmod +x scripts/deploy_px4_model.sh
./scripts/deploy_px4_model.sh --px4-dir ~/PX4-Autopilot

# Rebuild PX4 with new models
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth_mono

# Build ROS2 package
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash

# Step 7: Setup environment
echo -e "${YELLOW}[7/7] Setting up environment...${NC}"
if ! grep -q "PX4_AUTOPILOT_PATH" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# SES598 Assignment 3 setup" >> ~/.bashrc
    echo "export PX4_AUTOPILOT_PATH=~/PX4-Autopilot" >> ~/.bashrc
    echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/terrain_mapping_drone_control/models" >> ~/.bashrc
fi

echo -e "${GREEN}=== Setup Complete! ===${NC}"
echo ""
echo "Next steps:"
echo "1. Restart your terminal or run: source ~/.bashrc"
echo "2. Run: ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py"
```

Make executable and run:

```bash
chmod +x setup_assignment3.sh
./setup_assignment3.sh
```

**Success check:** Gazebo opens with drone and terrain, drone takes off and begins spiral search pattern.

---

## Common Issues & Quick Fixes

### Issue: "Package not found"
```bash
# Rebuild workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Issue: Gazebo won't start
```bash
# Clear cache
rm -rf ~/.gz/sim/
```

### Issue: Multiple ROS2 distributions causing conflicts
```bash
# Use only one
unset ROS_DISTRO
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Issue: "No module named 'X'" for Python package
```bash
pip3 install --user PACKAGE_NAME
```

### Issue: PX4 build fails
```bash
cd ~/PX4-Autopilot
make clean
make distclean
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500
```

---

## Essential Commands Cheatsheet

### ROS2 Basics
```bash
# List packages
ros2 pkg list

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /topic_name

# List nodes
ros2 node list

# Node info
ros2 node info /node_name

# Check message type
ros2 interface show PACKAGE/msg/MessageType
```

### Debugging
```bash
# Run with debug output
ros2 run PACKAGE NODE --ros-args --log-level debug

# Monitor topic frequency
ros2 topic hz /topic_name

# Check topic bandwidth
ros2 topic bw /topic_name

# Record data
ros2 bag record -a -o my_recording
```

### Gazebo
```bash
# List models
gz model --list

# List topics
gz topic -l

# Echo topic
gz topic -e -t /topic_name
```

### Workspace Management
```bash
# Build specific package
colcon build --packages-select PACKAGE_NAME --symlink-install

# Clean build
rm -rf build/ install/ log/
colcon build

# Test package
colcon test --packages-select PACKAGE_NAME
```

---

## Getting Help

### Before Asking for Help:

1. **Check logs:**
   ```bash
   # ROS2 logs
   cat ~/.ros/log/latest/PACKAGE_NAME/stdout.log
   
   # Gazebo logs
   cat ~/.gz/sim/logs/latest/server.log
   
   # PX4 logs (Assignment 3)
   cat ~/PX4-Autopilot/logs/latest/px4.log
   ```

2. **Run system check:**
   ```bash
   # Check all ROS2 nodes
   ros2 node list
   
   # Check all topics
   ros2 topic list
   
   # Check workspace
   echo $ROS_PACKAGE_PATH
   ```

3. **Search existing issues:**
   - Check course repository issues
   - Search ROS2 documentation
   - Check PX4 documentation (Assignment 3)

### When Reporting Issues:

Include:
- Operating system and version
- ROS2 distribution
- Assignment number
- Error messages (full text)
- Steps to reproduce
- Log files

---

## Performance Tips

### For Slow Systems:

```bash
# Reduce Gazebo graphics
export LIBGL_ALWAYS_SOFTWARE=1

# Run Gazebo headless (no GUI)
gz sim -s world.sdf

# Limit ROS2 logging
export RCUTILS_LOGGING_USE_STDOUT=0
```

### For Faster Development:

```bash
# Always use --symlink-install
colcon build --symlink-install

# Build only changed packages
colcon build --packages-select PACKAGE_NAME

# Parallel builds
colcon build --parallel-workers 4
```

---

## Resource Requirements

| Assignment | Min RAM | Recommended RAM | Disk Space | GPU |
|------------|---------|-----------------|------------|-----|
| Assignment 1 | 4GB | 8GB | 1GB | No |
| Assignment 2 | 4GB | 8GB | 5GB | Optional |
| Assignment 3 | 8GB | 16GB | 20GB | Recommended |

---

## Next Steps

1. **Start with Assignment 1** - It's the easiest and will help you get familiar with ROS2
2. **Complete the tutorials** in the samples/ directory
3. **Join the course forum** for discussions and help
4. **Read the detailed assignment READMEs** for grading criteria

---

**Need more help?** See [README_IMPROVEMENTS.md](assignments/README_IMPROVEMENTS.md) for detailed troubleshooting and advanced topics.

**Course Website:** [https://deepgis.org/dreamslab/ses598/](https://deepgis.org/dreamslab/ses598/)

