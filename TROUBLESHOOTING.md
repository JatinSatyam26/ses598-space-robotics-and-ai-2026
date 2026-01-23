# Troubleshooting Guide - SES598 Space Robotics and AI

This guide covers common issues across all assignments with solutions, organized by category.

---

## Table of Contents

1. [ROS2 Installation & Environment Issues](#ros2-installation--environment-issues)
2. [Workspace Build Issues](#workspace-build-issues)
3. [Gazebo Issues](#gazebo-issues)
4. [Python & Dependencies Issues](#python--dependencies-issues)
5. [Assignment 1: Boustrophedon Navigator](#assignment-1-boustrophedon-navigator)
6. [Assignment 2: Cart-Pole Control](#assignment-2-cart-pole-control)
7. [Assignment 3: Drone Control](#assignment-3-drone-control)
8. [Performance & Resource Issues](#performance--resource-issues)
9. [Git & Repository Issues](#git--repository-issues)

---

## ROS2 Installation & Environment Issues

### ❌ "ros2: command not found"

**Cause:** ROS2 not installed or not sourced

**Solution:**
```bash
# Check if ROS2 is installed
ls /opt/ros/

# Source ROS2
source /opt/ros/humble/setup.bash  # or jazzy, iron

# Add to .bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### ❌ "Package 'X' not found"

**Cause:** Workspace not sourced or package not built

**Solution:**
```bash
# 1. Source workspace
source ~/ros2_ws/install/setup.bash

# 2. If still not found, rebuild
cd ~/ros2_ws
colcon build --packages-select PACKAGE_NAME --symlink-install

# 3. Source again
source install/setup.bash

# 4. Verify
ros2 pkg prefix PACKAGE_NAME
```

### ❌ Multiple ROS2 distributions conflict

**Symptoms:** Weird errors, packages not found, version mismatches

**Solution:**
```bash
# 1. Clear environment
unset ROS_DISTRO
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# 2. Source only one ROS2 distribution
source /opt/ros/humble/setup.bash

# 3. Clean workspace and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build

# 4. Source workspace
source install/setup.bash
```

### ❌ "setuptools" or "colcon" errors

**Solution:**
```bash
# Update setuptools and colcon
pip3 install --upgrade setuptools
sudo apt update
sudo apt install --reinstall python3-colcon-common-extensions
```

---

## Workspace Build Issues

### ❌ "CMake Error" during build

**Solution:**
```bash
# 1. Clean build artifacts
cd ~/ros2_ws
rm -rf build/ install/ log/

# 2. Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# 3. Rebuild with verbose output
colcon build --event-handlers console_direct+
```

### ❌ "No package named 'X'" during build (Python)

**Solution:**
```bash
# Install missing Python package
pip3 install PACKAGE_NAME

# For ROS2 packages
sudo apt install ros-$ROS_DISTRO-PACKAGE-NAME

# Update rosdep and install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### ❌ Symlink install not working

**Symptoms:** Changes to Python files don't take effect

**Solution:**
```bash
# 1. Clean install directory
cd ~/ros2_ws
rm -rf install/

# 2. Rebuild with symlink-install
colcon build --symlink-install

# 3. For specific package
colcon build --packages-select PACKAGE_NAME --symlink-install

# Note: C++ changes always require rebuild
```

### ❌ "Permission denied" errors

**Solution:**
```bash
# Fix workspace permissions
cd ~/ros2_ws
sudo chown -R $USER:$USER .

# Fix script permissions
find . -name "*.py" -exec chmod +x {} \;
find . -name "*.sh" -exec chmod +x {} \;
```

---

## Gazebo Issues

### ❌ "gz: command not found"

**Cause:** Gazebo not installed

**Solution:**
```bash
# For Ubuntu 22.04
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-garden

# For Ubuntu 24.04
sudo apt install gz-harmonic

# Verify
gz sim --version
```

### ❌ Gazebo crashes on startup

**Solution:**
```bash
# 1. Clear Gazebo cache
rm -rf ~/.gz/

# 2. Update Gazebo
sudo apt update
sudo apt upgrade gz-garden  # or gz-harmonic

# 3. Check graphics drivers
glxinfo | grep "OpenGL"

# 4. If NVIDIA GPU, update drivers
sudo ubuntu-drivers autoinstall

# 5. If still failing, use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

### ❌ Gazebo freezes or very slow

**Solution:**
```bash
# 1. Reduce graphics quality
export LIBGL_ALWAYS_SOFTWARE=1

# 2. Run headless (server only, no GUI)
gz sim -s world.sdf

# 3. Close other applications
# 4. Check GPU usage
nvidia-smi  # For NVIDIA GPUs
top  # Check CPU usage

# 5. Reduce physics update rate in world file
# Edit .sdf file: <max_step_size>0.01</max_step_size>
```

### ❌ Models don't appear in Gazebo

**Solution:**
```bash
# 1. Check model path is set
echo $GZ_SIM_RESOURCE_PATH

# 2. Add model path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/models

# 3. List available models
gz model --list

# 4. Verify model files exist
ls -la /path/to/models/MODEL_NAME/

# 5. Check model.sdf syntax
gz sdf --check /path/to/models/MODEL_NAME/model.sdf
```

### ❌ ROS2-Gazebo bridge not working

**Solution:**
```bash
# 1. Check bridge package is installed
ros2 pkg list | grep ros_gz

# 2. Install if missing
sudo apt install ros-$ROS_DISTRO-ros-gz-bridge

# 3. Check bridge is running
ros2 node list | grep bridge

# 4. List Gazebo topics
gz topic -l

# 5. List ROS2 topics
ros2 topic list

# 6. Test bridge manually
ros2 run ros_gz_bridge parameter_bridge /topic@ros_msg_type]gz_msg_type
```

---

## Python & Dependencies Issues

### ❌ "No module named 'numpy'" (or scipy, matplotlib, etc.)

**Solution:**
```bash
# Install for current user
pip3 install --user numpy scipy matplotlib

# Or install system-wide
sudo apt install python3-numpy python3-scipy python3-matplotlib

# Check Python version matches
python3 --version
which python3

# If multiple Python versions:
python3.10 -m pip install numpy  # Use specific version
```

### ❌ "No module named 'cv2'"

**Solution:**
```bash
# Install OpenCV
pip3 install opencv-python

# Or use ROS2 package
sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv
```

### ❌ "No module named 'torch'"

**For Assignment 2 DQN extra credit**

**Solution:**
```bash
# Install PyTorch (CPU version)
pip3 install torch

# For GPU support (requires CUDA)
pip3 install torch --index-url https://download.pytorch.org/whl/cu118

# Verify
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

### ❌ "No module named 'gymnasium'"

**For Assignment 2 DQN extra credit**

**Solution:**
```bash
# Install gymnasium (replaces deprecated 'gym')
pip3 install gymnasium

# Do NOT install 'gym' - it's outdated
```

### ❌ Python packages installed but not found

**Cause:** Wrong Python version or path issues

**Solution:**
```bash
# Check which Python is being used
which python3

# Check pip install location
pip3 show numpy

# Install for specific Python version
python3.10 -m pip install numpy

# Add user site-packages to path
export PYTHONPATH=$PYTHONPATH:~/.local/lib/python3.10/site-packages
```

---

## Assignment 1: Boustrophedon Navigator

### ❌ Turtlesim window doesn't open

**Solution:**
```bash
# Test turtlesim separately
ros2 run turtlesim turtlesim_node

# Check X display
echo $DISPLAY

# If WSL, install X server (VcXsrv or Xming)
export DISPLAY=:0

# Check Qt installation
sudo apt install python3-pyqt5
```

### ❌ rqt_reconfigure doesn't show parameters

**Solution:**
```bash
# 1. Check if node is running
ros2 node list

# 2. List parameters for node
ros2 param list /boustrophedon_controller

# 3. Restart rqt_reconfigure
killall rqt_reconfigure
ros2 run rqt_reconfigure rqt_reconfigure

# 4. Use command line to set params
ros2 param set /boustrophedon_controller Kp_linear 2.0
```

### ❌ Turtle moves erratically or doesn't follow path

**Solution:**
```bash
# 1. Check for multiple controller instances
ros2 node list
# Kill all ROS2 processes if duplicates found
killall -9 ros2

# 2. Reset turtlesim
ros2 service call /reset std_srvs/srv/Empty

# 3. Check controller parameters
ros2 param get /boustrophedon_controller Kp_linear
ros2 param get /boustrophedon_controller Kd_linear

# 4. Reduce gains if oscillating
ros2 param set /boustrophedon_controller Kp_linear 1.0
```

### ❌ Cross-track error topic not publishing

**Solution:**
```bash
# Check topic exists
ros2 topic list | grep cross_track

# Check topic type
ros2 topic info /cross_track_error

# Echo topic
ros2 topic echo /cross_track_error

# Check node is running
ros2 node info /boustrophedon_controller
```

---

## Assignment 2: Cart-Pole Control

### ❌ Cart-pole doesn't spawn in Gazebo

**Solution:**
```bash
# 1. Check URDF file
check_urdf ~/ros2_ws/install/cart_pole_optimal_control/share/cart_pole_optimal_control/models/cart_pole/model.urdf

# 2. Check robot_description topic
ros2 topic echo /robot_description --once

# 3. Check spawn node is running
ros2 node list | grep create

# 4. Check Gazebo logs
cat ~/.gz/sim/logs/latest/server.log | grep error
```

### ❌ "Scipy linalg error" when computing LQR gain

**Solution:**
```bash
# 1. Update scipy
pip3 install --upgrade scipy

# 2. Check Q and R matrices in code
# Q must be positive semi-definite
# R must be positive definite

# 3. Verify matrix dimensions match state/control dimensions
# For cart-pole: Q should be 4x4, R should be 1x1
```

### ❌ Cart immediately fails or goes out of bounds

**Solution:**
```bash
# 1. Check controller is publishing
ros2 topic hz /model/cart_pole/joint/cart_to_base/cmd_force

# 2. Check joint states are being received
ros2 topic echo /joint_states

# 3. Verify LQR gain matrix is computed
# Check terminal output for "LQR gain matrix: ..."

# 4. Reduce earthquake force for testing
ros2 param set /earthquake_force_generator base_amplitude 5.0
```

### ❌ RViz doesn't show cart-pole

**Solution:**
```bash
# 1. Check RViz configuration
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py

# 2. Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf to see TF tree

# 3. Add RobotModel display manually in RViz
# Fixed Frame: world
# Robot Description: robot_description

# 4. Check robot_state_publisher is running
ros2 node list | grep robot_state_publisher
```

### ❌ DQN training fails or doesn't learn

**Solution:**
```bash
# 1. Verify gymnasium is installed (not 'gym')
pip3 list | grep gymnasium

# 2. Check PyTorch is working
python3 -c "import torch; print(torch.randn(2,2))"

# 3. Reduce number of episodes for testing
# Edit dqn_train.py: num_episodes = 1000 (instead of 15000)

# 4. Check reward function is working
# Add debug prints in training loop

# 5. Monitor training progress
# Check that epsilon is decreasing
# Check that loss is being calculated
```

---

## Assignment 3: Drone Control

### ❌ PX4 build fails

**Solution:**
```bash
# 1. Update submodules
cd ~/PX4-Autopilot
git submodule update --init --recursive

# 2. Clean build
make clean
make distclean

# 3. Reinstall dependencies
bash ./Tools/setup/ubuntu.sh

# 4. Try building again
make px4_sitl gz_x500

# 5. If still failing, check logs
cat /tmp/px4_build_*.log
```

### ❌ "Model x500_depth_mono not found"

**Solution:**
```bash
# 1. Check if model exists
ls ~/PX4-Autopilot/Tools/simulation/gz/models/ | grep x500_depth_mono

# 2. If not found, redeploy from assignment
cd ~/ros2_ws/src/terrain_mapping_drone_control
./scripts/deploy_px4_model.sh --px4-dir ~/PX4-Autopilot

# 3. Verify deployment
ls ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth_mono/

# 4. Rebuild PX4
cd ~/PX4-Autopilot
make clean
make px4_sitl gz_x500_depth_mono
```

### ❌ "Airframe 4022 not found"

**Solution:**
```bash
# 1. Check airframe file
ls ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ | grep 4022

# 2. Redeploy
cd ~/ros2_ws/src/terrain_mapping_drone_control
./scripts/deploy_px4_model.sh --px4-dir ~/PX4-Autopilot

# 3. Check CMakeLists.txt includes new airframe
cat ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt | grep 4022

# 4. Rebuild PX4
cd ~/PX4-Autopilot
make clean
make px4_sitl gz_x500_depth_mono
```

### ❌ No PX4 topics in ROS2 (/fmu/* topics missing)

**Solution:**
```bash
# 1. Check PX4 is running
ps aux | grep px4

# 2. Check px4_msgs package
ros2 interface list | grep px4_msgs

# 3. Install px4_msgs if missing
sudo apt install ros-$ROS_DISTRO-px4-msgs

# 4. Check DDS configuration
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 5. Restart PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth_mono

# 6. Wait 10-15 seconds, then check topics
ros2 topic list | grep fmu
```

### ❌ Drone doesn't takeoff or respond to commands

**Solution:**
```bash
# 1. Check vehicle status
ros2 topic echo /fmu/out/vehicle_status --once

# 2. Check trajectory setpoints are being published
ros2 topic hz /fmu/in/trajectory_setpoint

# 3. Check node is running
ros2 node list | grep spiral

# 4. Check for arming rejection
# Look for error messages in PX4 terminal

# 5. Verify offboard mode is enabled
ros2 topic echo /fmu/out/vehicle_control_mode --once
# offboard should be true
```

### ❌ RTAB-Map crashes or shows black screen

**Solution:**
```bash
# 1. Check camera topics
ros2 topic list | grep camera
ros2 topic hz /camera/image
ros2 topic hz /camera/depth

# 2. Reduce RTAB-Map parameters
# Edit launch file:
# 'Vis/MaxFeatures': '200'  # instead of 1000
# 'Mem/IncrementalMemory': 'true'

# 3. Clear RTAB-Map database
rm ~/.ros/rtabmap.db

# 4. Check RTAB-Map logs
cat ~/.ros/log/latest/rtabmap/stdout.log

# 5. Run RTAB-Map separately for testing
ros2 launch rtabmap_ros rtabmap.launch.py
```

### ❌ Terrain or cylinder models don't appear

**Solution:**
```bash
# 1. Set Gazebo resource path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/terrain_mapping_drone_control/models

# 2. Add to .bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/terrain_mapping_drone_control/models' >> ~/.bashrc

# 3. Verify models exist
ls ~/ros2_ws/src/terrain_mapping_drone_control/models/
ls ~/ros2_ws/src/terrain_mapping_drone_control/models/terrain/
ls ~/ros2_ws/src/terrain_mapping_drone_control/models/cylinder/

# 4. Check model.sdf files are valid
gz sdf --check ~/ros2_ws/src/terrain_mapping_drone_control/models/terrain/model.sdf
```

---

## Performance & Resource Issues

### ❌ System runs out of memory

**Solution:**
```bash
# 1. Check memory usage
free -h

# 2. Close unnecessary applications

# 3. Reduce Gazebo resource usage
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -s  # Headless mode

# 4. Limit ROS2 logging
export RCUTILS_LOGGING_USE_STDOUT=0

# 5. For RTAB-Map, reduce memory:
# 'Mem/IncrementalMemory': 'true'
# 'Mem/RehearsalSimilarity': '0.30'
```

### ❌ Simulation runs very slowly

**Solution:**
```bash
# 1. Close other applications
# 2. Update graphics drivers
sudo ubuntu-drivers autoinstall

# 3. Use software rendering
export LIBGL_ALWAYS_SOFTWARE=1

# 4. Reduce physics update rate
# Edit world .sdf file
# <max_step_size>0.01</max_step_size>  # Increase to 0.02

# 5. Disable GUI rendering
gz sim -s world.sdf  # Server only

# 6. Reduce camera resolution (Assignment 3)
# Edit camera sensor in model.sdf
# <width>320</width>  # Instead of 640
# <height>240</height>  # Instead of 480
```

### ❌ Disk space running out

**Solution:**
```bash
# 1. Check disk usage
df -h
du -sh ~/.ros/log/*
du -sh ~/.gz/

# 2. Clean ROS2 logs
rm -rf ~/.ros/log/*

# 3. Clean Gazebo cache
rm -rf ~/.gz/sim/

# 4. Clean workspace build artifacts
cd ~/ros2_ws
rm -rf build/ log/

# 5. Clean PX4 logs (Assignment 3)
cd ~/PX4-Autopilot
rm -rf logs/*

# 6. Clean apt cache
sudo apt clean
sudo apt autoremove
```

---

## Git & Repository Issues

### ❌ "Permission denied" when cloning

**Solution:**
```bash
# Use HTTPS instead of SSH
git clone https://github.com/USERNAME/REPO.git

# Or setup SSH keys
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub
# Add to GitHub Settings > SSH Keys
```

### ❌ Can't pull updates from upstream

**Solution:**
```bash
# 1. Add upstream remote (if not done)
cd ~/ses598-space-robotics-and-ai-2026
git remote add upstream https://github.com/DREAMS-lab/ses598-space-robotics-and-ai-2026.git

# 2. Fetch upstream
git fetch upstream

# 3. Merge upstream changes
git checkout main
git merge upstream/main

# 4. If merge conflicts
git status  # Shows conflicted files
# Edit conflicted files, resolve conflicts
git add .
git commit -m "Resolved merge conflicts"

# 5. Push to your fork
git push origin main
```

### ❌ Submodule errors

**Solution:**
```bash
# Update all submodules
git submodule update --init --recursive

# If stuck, force update
git submodule foreach git pull origin main

# Reset submodule to clean state
git submodule foreach --recursive git clean -xfd
git submodule foreach --recursive git reset --hard
git submodule update --init --recursive
```

### ❌ Modified files preventing pull

**Solution:**
```bash
# Stash local changes
git stash

# Pull updates
git pull

# Reapply local changes
git stash pop

# Or discard local changes (be careful!)
git reset --hard origin/main
```

---

## General Debugging Workflow

When something doesn't work, follow this systematic approach:

### 1. Gather Information
```bash
# Check what's running
ros2 node list
ros2 topic list
ps aux | grep ros2
ps aux | grep gz

# Check logs
cat ~/.ros/log/latest/PACKAGE/stdout.log
cat ~/.gz/sim/logs/latest/server.log

# Check environment
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH
echo $PYTHONPATH
```

### 2. Isolate the Problem
```bash
# Test components individually
# For example, if launch file fails:

# 1. Test Gazebo alone
gz sim empty.sdf

# 2. Test ROS2 package alone
ros2 run PACKAGE NODE

# 3. Test launch file with minimal components
# Comment out nodes in launch file one by one
```

### 3. Search for Solutions
1. Check this troubleshooting guide
2. Search course repository issues
3. Google the exact error message
4. Check ROS2/Gazebo/PX4 documentation
5. Ask on course forum with details

### 4. Report Issues

When reporting issues, include:

```bash
# System information
cat /etc/os-release
uname -a
ros2 --version
gz sim --version  # If applicable
python3 --version

# Environment
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH

# Package information
ros2 pkg list | grep PACKAGE_NAME
colcon list

# Error logs (last 50 lines)
cat ~/.ros/log/latest/PACKAGE/stdout.log | tail -n 50

# Steps to reproduce
# 1. ...
# 2. ...
# 3. Error occurs
```

---

## Prevention Tips

### Best Practices to Avoid Issues:

1. **Always use --symlink-install**
   ```bash
   colcon build --symlink-install
   ```

2. **Source workspace in every new terminal**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **Keep dependencies updated**
   ```bash
   sudo apt update
   sudo apt upgrade
   pip3 install --upgrade pip
   ```

4. **Use version control**
   ```bash
   git add .
   git commit -m "Describe changes"
   git push
   ```

5. **Clean builds periodically**
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build --symlink-install
   ```

6. **Monitor resources**
   ```bash
   htop  # Monitor CPU/RAM
   df -h  # Monitor disk space
   ```

7. **Keep backups**
   ```bash
   # Backup workspace
   tar -czf ros2_ws_backup_$(date +%Y%m%d).tar.gz ~/ros2_ws/src
   ```

---

## Still Having Issues?

### Pre-Submission Checklist:

Before asking for help, verify you've tried:

- [ ] Checked this troubleshooting guide
- [ ] Searched existing issues on GitHub
- [ ] Cleaned and rebuilt workspace
- [ ] Checked all environment variables
- [ ] Tested components individually
- [ ] Checked log files for errors
- [ ] Verified all dependencies are installed
- [ ] Restarted your computer

### Where to Get Help:

1. **Course Forum:** Post with full error details
2. **Office Hours:** Bring your system for live debugging
3. **GitHub Issues:** For code-related problems
4. **Email:** For private/urgent issues

### What to Include When Asking for Help:

1. Operating system and version
2. ROS2 distribution
3. Assignment number and specific step
4. Full error message (not a screenshot if possible)
5. What you've already tried
6. Relevant log files
7. Output of verification commands

---

**Document Version:** 1.0  
**Last Updated:** January 2026  
**Course:** SES598 Space Robotics and AI  
**Institution:** Arizona State University

