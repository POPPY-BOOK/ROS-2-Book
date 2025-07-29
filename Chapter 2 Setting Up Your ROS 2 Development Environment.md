The chapter provides detailed instructions and insights on setting up a ROS 2 development environment, with a focus on using Ubuntu Linux as the recommended platform. Here are the key coding commands and steps extracted from the chapter:

### Platform Selection and Installation

#### Ubuntu Linux Installation

1. **Update and Install Prerequisites:**
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   ```

2. **Add ROS 2 Repository:**
   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Update Package Index:**
   ```bash
   sudo apt update
   ```

4. **Install ROS 2:**
   ```bash
   sudo apt install ros-humble-desktop
   ```

5. **Install Development Tools:**
   ```bash
   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-rosdep python3-vcstool
   ```

6. **Initialize `rosdep`:**
   ```bash
   sudo rosdep init
   rosdep update
   ```

7. **Set Up Environment:**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Verification and Troubleshooting

1. **Verify ROS 2 Installation:**
   ```bash
   ros2 --version
   ros2 --help
   ros2 pkg list
   ```

2. **Test Node Communication:**
   ```bash
   ros2 run demo_nodes_cpp talker
   ros2 run demo_nodes_py listener
   ```

3. **Check Build Tools:**
   ```bash
   colcon --version
   ```

4. **Test GUI Tools:**
   ```bash
   ros2 run rqt_graph rqt_graph
   ```

5. **Verify Python Integration:**
   ```bash
   python3 -c "import rclpy; print('Python ROS 2 bindings work')"
   ```

### Troubleshooting Common Issues

1. **GPG Key Problems:**
   ```bash
   sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
   # Repeat the key installation step
   ```

2. **Package Dependency Conflicts:**
   ```bash
   sudo apt update
   sudo apt upgrade
   sudo apt autoremove
   sudo apt autoclean
   ```

3. **`rosdep` Initialization Failures:**
   ```bash
   sudo rm -rf /etc/ros/rosdep/
   sudo rosdep init
   rosdep update
   ```

4. **Environment Setup Issues:**
   Ensure your `~/.bashrc` contains the source line and that you've sourced it in your current terminal.

5. **DDS Discovery Problems:**
   ```bash
   sudo ufw disable
   # Test your nodes
   sudo ufw enable
   ```

6. **Performance Issues:**
   ```bash
   echo $RMW_IMPLEMENTATION
   sudo apt install ros-humble-rmw-cyclonedx-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
   ```

7. **Permission Problems:**
   ```bash
   sudo usermod -a -G dialout $USER
   ```

These commands and steps should help you set up and verify your ROS 2 development environment on Ubuntu Linux, as well as troubleshoot common issues that may arise during the installation process.