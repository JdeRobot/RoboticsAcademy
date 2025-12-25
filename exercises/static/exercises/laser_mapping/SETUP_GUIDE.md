# Laser Mapping Exercise - Setup Guide

This guide provides step-by-step instructions for setting up and running the Laser Mapping exercise.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Exercise](#running-the-exercise)
- [Troubleshooting](#troubleshooting)
- [Development Setup](#development-setup)

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **RAM**: Minimum 4GB, recommended 8GB
- **Storage**: At least 10GB free space
- **Graphics**: OpenGL 3.3 support for Gazebo simulation

### Software Dependencies
- **ROS2 Humble**: Robot Operating System 2
- **Gazebo**: 3D robot simulation environment
- **Python 3.8+**: Programming language
- **Docker** (optional): For containerized deployment

## Installation

### Method 1: Quick Start (Recommended)

The easiest way to run the exercise is through the RoboticsAcademy web platform:

1. **Access the Platform**
   ```bash
   curl -s https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/scripts/run_academy.sh | sudo bash
   ```

2. **Open Web Interface**
   - Navigate to `http://localhost:7164` in your browser
   - Find "Laser Mapping" under Mobile Robots exercises
   - Click "Launch Exercise"

### Method 2: Docker Installation

1. **Install Docker**
   ```bash
   # Ubuntu/Debian
   sudo apt update
   sudo apt install docker.io docker-compose
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. **Clone Repository**
   ```bash
   git clone https://github.com/JdeRobot/RoboticsAcademy.git
   cd RoboticsAcademy
   ```

3. **Run with Docker Compose**
   ```bash
   # For CPU-only systems
   docker-compose -f compose_cfg/user_humble_cpu.yaml up
   
   # For systems with NVIDIA GPU
   docker-compose -f compose_cfg/user_humble_gpu.yaml up
   ```

### Method 3: Native Installation

1. **Install ROS2 Humble**
   ```bash
   # Add ROS2 repository
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   
   # Add ROS2 GPG key
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   # Add repository to sources list
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **Install Additional Dependencies**
   ```bash
   # Gazebo simulation
   sudo apt install gazebo
   sudo apt install ros-humble-gazebo-*
   
   # TurtleBot3 packages
   sudo apt install ros-humble-turtlebot3*
   
   # Navigation and SLAM packages
   sudo apt install ros-humble-navigation2
   sudo apt install ros-humble-nav2-bringup
   sudo apt install ros-humble-slam-toolbox
   
   # Python dependencies
   sudo apt install python3-pip
   pip3 install numpy opencv-python matplotlib
   ```

3. **Setup Environment**
   ```bash
   # Add to ~/.bashrc
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
   echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Clone and Build RoboticsAcademy**
   ```bash
   # Clone repository
   git clone https://github.com/JdeRobot/RoboticsAcademy.git
   cd RoboticsAcademy
   
   # Install Python dependencies
   pip3 install -r requirements.txt
   
   # Setup database
   python3 manage.py migrate
   python3 manage.py collectstatic
   ```

## Running the Exercise

### Web Interface Method

1. **Start the Academy**
   ```bash
   cd RoboticsAcademy
   python3 manage.py runserver 0.0.0.0:8000
   ```

2. **Access Exercise**
   - Open browser to `http://localhost:8000`
   - Navigate to Exercises → Mobile Robots → Laser Mapping
   - Click "Launch Exercise"

3. **Wait for Initialization**
   - The simulation environment will load (may take 1-2 minutes)
   - You'll see the Gazebo simulator and web interface
   - Green "Ready" indicator shows when exercise is ready

### Direct Launch Method

1. **Terminal 1: Start Gazebo Simulation**
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Terminal 2: Run Exercise Code**
   ```bash
   cd RoboticsAcademy/exercises/static/exercises/laser_mapping/python_template/ros2_humble
   python3 exercise.py
   ```

3. **Terminal 3: Start Web GUI**
   ```bash
   cd RoboticsAcademy/exercises/static/exercises/laser_mapping/python_template/ros2_humble
   python3 WebGUI.py
   ```

## Exercise Interface

### Web Interface Components

1. **Code Editor**
   - Edit your algorithm in `MyAlgorithm.py`
   - Syntax highlighting and error detection
   - Auto-save functionality

2. **Simulation View**
   - Real-time Gazebo simulation
   - Robot movement visualization
   - Environment interaction

3. **Map Display**
   - Shows your generated map
   - Real-time updates as robot explores
   - Robot position and path tracking

4. **Control Panel**
   - Start/Stop/Reset buttons
   - Frequency control
   - Console output

### Available Environments

The exercise includes multiple warehouse environments:

1. **Small Warehouse** (default)
   - Compact environment for quick testing
   - Simple rectangular layout
   - Good for algorithm development

2. **Large Warehouse**
   - Complex multi-room environment
   - More challenging for mapping
   - Tests algorithm robustness

## Configuration

### Exercise Parameters

Edit the configuration in `exercise.py`:

```python
# Robot parameters
ROBOT_MODEL = "turtlebot3_waffle"
MAX_LINEAR_VELOCITY = 0.5  # m/s
MAX_ANGULAR_VELOCITY = 1.0  # rad/s

# Simulation parameters
SIMULATION_FREQUENCY = 30  # Hz
GUI_UPDATE_FREQUENCY = 10  # Hz

# Map parameters
MAP_WIDTH = 1500   # pixels
MAP_HEIGHT = 970   # pixels
MAP_RESOLUTION = 0.05  # meters/pixel
```

### Environment Selection

Change the simulation world:

```bash
# Small warehouse (default)
export GAZEBO_WORLD="small_warehouse.world"

# Large warehouse
export GAZEBO_WORLD="warehouse.world"

# Custom world
export GAZEBO_WORLD="/path/to/your/world.world"
```

## Troubleshooting

### Common Issues

#### 1. Exercise Won't Start
**Symptoms**: Blank screen, loading forever
**Solutions**:
```bash
# Check if ROS2 is sourced
source /opt/ros/humble/setup.bash

# Verify TurtleBot3 model is set
echo $TURTLEBOT3_MODEL

# Check if Gazebo is running
ps aux | grep gazebo

# Restart the exercise
pkill -f gazebo
pkill -f python3
# Then restart
```

#### 2. No Laser Data
**Symptoms**: `getLaserData()` returns empty
**Solutions**:
```bash
# Check laser topic
ros2 topic list | grep scan
ros2 topic echo /turtlebot3/laser/scan

# Verify simulation is running
ros2 node list
```

#### 3. Map Not Displaying
**Symptoms**: Map window shows nothing
**Solutions**:
```python
# Check map dimensions
print(f"Map shape: {my_map.shape}")  # Should be (970, 1500)

# Verify data type
print(f"Map dtype: {my_map.dtype}")  # Should be uint8

# Check value range
print(f"Map range: {my_map.min()} to {my_map.max()}")  # Should be 0-255
```

#### 4. Robot Not Moving
**Symptoms**: Robot stays in place despite commands
**Solutions**:
```python
# Test motor commands
from HAL import setV, setW
setV(0.1)  # Small forward velocity
setW(0.0)  # No rotation

# Check if commands are being sent
print("Sending velocity commands...")
```

#### 5. Performance Issues
**Symptoms**: Slow simulation, lag
**Solutions**:
```bash
# Reduce simulation quality
export GAZEBO_MASTER_URI=http://localhost:11345
gazebo --verbose --pause

# Lower GUI update frequency
# In WebGUI.py, change:
# super().__init__(host, freq=10.0)  # Lower frequency
```

### Log Files

Check log files for detailed error information:

```bash
# ROS2 logs
ros2 log list
ros2 log view

# Gazebo logs
ls ~/.gazebo/log/

# Exercise logs
tail -f /tmp/robotics_academy.log
```

### Getting Help

1. **Check Documentation**
   - README.md for exercise overview
   - API_REFERENCE.md for function details
   - This SETUP_GUIDE.md for installation

2. **Community Support**
   - GitHub Issues: https://github.com/JdeRobot/RoboticsAcademy/issues
   - ROS2 Community: https://discourse.ros.org/
   - JdeRobot Forum: https://jderobot.github.io/

3. **Debug Mode**
   ```python
   # Enable debug output in your code
   import logging
   logging.basicConfig(level=logging.DEBUG)
   
   # Add debug prints
   print(f"Debug: Robot pose = {getPose3d()}")
   print(f"Debug: Laser data length = {len(getLaserData().values)}")
   ```

## Development Setup

### For Contributors

1. **Fork and Clone**
   ```bash
   git clone https://github.com/YOUR_USERNAME/RoboticsAcademy.git
   cd RoboticsAcademy
   git remote add upstream https://github.com/JdeRobot/RoboticsAcademy.git
   ```

2. **Create Development Branch**
   ```bash
   git checkout -b feature/laser-mapping-improvements
   ```

3. **Install Development Dependencies**
   ```bash
   pip3 install -r requirements-dev.txt
   pre-commit install
   ```

4. **Run Tests**
   ```bash
   python3 -m pytest tests/
   ```

### Code Style

Follow the project coding standards:

```bash
# Format code
black exercises/static/exercises/laser_mapping/
isort exercises/static/exercises/laser_mapping/

# Check style
flake8 exercises/static/exercises/laser_mapping/
pylint exercises/static/exercises/laser_mapping/
```

### Testing Your Changes

1. **Unit Tests**
   ```bash
   cd exercises/static/exercises/laser_mapping/
   python3 -m pytest test_*.py
   ```

2. **Integration Tests**
   ```bash
   # Test with simulation
   python3 test_exercise_integration.py
   ```

3. **Manual Testing**
   - Test all exercise features
   - Verify documentation accuracy
   - Check different environments

## Performance Optimization

### System Optimization

1. **Graphics Settings**
   ```bash
   # Reduce Gazebo graphics quality
   export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
   gazebo --verbose --pause
   ```

2. **Resource Limits**
   ```bash
   # Limit CPU usage
   cpulimit -l 50 gazebo
   
   # Monitor resource usage
   htop
   nvidia-smi  # For GPU systems
   ```

### Algorithm Optimization

1. **Reduce Map Resolution**
   ```python
   # Use larger pixels for faster processing
   MAP_RESOLUTION = 0.1  # Instead of 0.05
   ```

2. **Limit Laser Processing**
   ```python
   # Process every Nth laser ray
   step = 5
   for i in range(0, len(ranges), step):
       # Process ranges[i]
   ```

3. **Optimize Map Updates**
   ```python
   # Update map less frequently
   if self.iteration_count % 10 == 0:
       self.update_map()
   ```

## Next Steps

After successful setup:

1. **Read the Exercise Documentation** (README.md)
2. **Study the API Reference** (API_REFERENCE.md)
3. **Implement Your Algorithm** (MyAlgorithm.py)
4. **Test and Debug** your solution
5. **Experiment** with different approaches
6. **Share Your Results** with the community

Good luck with your laser mapping implementation!