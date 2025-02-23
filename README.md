<div align="center">
    <img src="https://img.shields.io/github/stars/botforge-robotics/rio_ros2?style=social&logo=github" alt="Stars">&nbsp;
    <img src="https://img.shields.io/github/forks/botforge-robotics/rio_ros2?style=social&logo=github" alt="Forks">&nbsp;
    <img src="https://img.shields.io/github/issues/botforge-robotics/rio_ros2" alt="Issues">&nbsp;
    <img src="https://img.shields.io/github/repo-size/botforge-robotics/rio_ros2" alt="Repo Size">&nbsp;
    <img src="https://img.shields.io/github/license/botforge-robotics/rio_ros2?color=mit" alt="MIT License">
</div>

<!-- Start of Selection -->
<h2 align="center">Welcome to the Project RIO!</h2>
<!-- End of Selection -->

<!-- Start of Selection -->
<img src="./rio_description/images/rio.12a.jpg" style="border-radius: 15px; margin-bottom:10px;" alt="RIO">

🤖 **RIO Revolution** - Transform your smartphone into a fully-featured ROS2 robot! Utilize a wide array of built-in mobile sensors, including the Accelerometer, Gyro, Compass, GPS, NFC, IR, Ambient Light, Fingerprint Scanner, Cameras, and Mic/Speaker, as ROS2 topics, services, and actions. With the integration of Lidar, we can enable autonomously navigating companion robots that express emotions through animated facial expressions, while also extending functionality with our custom hardware platform.

### 📱 Mobile Core Features

- 📡 15+ built-in sensors as ROS2 interfaces
- 👁️ Programmable facial expressions displayed on the screen, enabling the robot to engage in conversation using its microphone and speaker.
- 🔌 Unified mobile-to-robot communication

<div style="display: flex; flex-wrap: wrap; gap: 15px; justify-content: center; margin: 25px 0; max-width: 800px; margin-left: auto; margin-right: auto;">
    <div style="flex: 0 0 calc(50% - 15px); box-sizing: border-box;">
        <img src="./rio_description/images/connectionScreen.jpeg" alt="Connection Interface" style=" border-radius: 10px; max-width: 100%;">
    </div>
    <div style="flex: 0 0 calc(50% - 15px); box-sizing: border-box;">
        <img src="./rio_description/images/listening.jpeg" alt="Listening Mode" style=" border-radius: 10px; max-width: 100%;">
    </div>
    <div style="flex: 0 0 calc(50% - 15px); box-sizing: border-box;">
        <img src="./rio_description/images/speaking.jpeg" alt="Speaking Mode" style=" border-radius: 10px; max-width: 100%;">
    </div>
    <div style="flex: 0 0 calc(50% - 15px); box-sizing: border-box;">
        <img src="./rio_description/images/settings1.jpeg" alt="Settings Page 1" style="border-radius: 10px;max-width: 100%;">
    </div>
    <div style="flex: 0 0 calc(50% - 15px); box-sizing: border-box;">
        <img src="./rio_description/images/settings2.jpeg" alt="Settings Page 2" style="border-radius: 10px; max-width: 100%;">
    </div>
    <div style="flex: 0 0 calc(50% - 15px); box-sizing: border-box;">
        <img src="./rio_description/images/settings3.jpeg" alt="Settings Page 3" style=" border-radius: 10px;  max-width: 100%;">
    </div>
</div>

### 🛠️ Hardware Expansion

- ⚡ RIO Control Board based on ESP32 (MicroROS compatible)
- 🔌 Easy-to-use plug-and-play ports:
  - 2x Motors (controlled via a 2-channel L293D driver)
  - 2x Quadrature Encoders
  - 2x Servos with a 5V 2A power output
  - 1x I2C port with 5V power supply
  - 1x WS2812B LED Strip
  - 1x Lidar sensor
  - 1x 7-12V DC power jack
  - Type-C programming connector
- 📡 Seamless integration with RPLIDAR A1M8 for enhanced mapping capabilities

<div style="text-align: center; margin: 20px;">
    <img src="./rio_description/images/pcb2.jpeg" alt="PCB Placeholder" style="max-width: 70%; height: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);">
</div>

## 🏛️ Rio Architecture

<div style="text-align: center; margin: 20px;">
    <img src="./rio_description/images/rio_architecture.jpg" alt="PCB Placeholder" style="max-width: 95%; height: auto; border-radius: 8px; ">
</div>

## ⚙️ Requirements

### Software Requirements

- [ROS 2 Iron Irwini](https://docs.ros.org/en/iron/Installation.html) (Recommended)
- [Ollama Installation](https://ollama.ai/download) (Local LLM Execution)
- [VSCode + PlatformIO](https://platformio.org/install/ide?install=vscode) (Firmware Flashing)
- [RIO Companion App](https://play.google.com/store/apps/details?id=com.botforge.rio) (Android 10+)
- [Rio Firmware](https://github.com/botforge-robotics/rio_firmware) (Flash RIO Pcb with this firmware)

### Hardware Requirements

- Assembled RIO Robot with Control PCB
- Smartphone with RIO App.
- PC with ROS2 Installed

## 🚀 Getting Started

### 1. Environment Setup

```bash
# 1.1 Source ROS installation
source /opt/ros/$ROS_DISTRO/setup.bash

# 1.2 Set up Micro-ROS workspace
mkdir -p ~/uros_ws/src
cd ~/uros_ws/src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
cd ~/uros_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash

# 1.3 Set up RIO workspace
mkdir -p ~/rio_ws/src
cd ~/rio_ws/src
git clone https://github.com/botforge-robotics/rio_ros2.git
cd ~/rio_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 2. Terminal Configuration

Add these to your `~/.bashrc` for automatic sourcing in new terminals:

```bash
# 2.1 Add ROS source
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# 2.2 Add Micro-ROS workspace source
echo "source ~/uros_ws/install/setup.bash" >> ~/.bashrc

# 2.3 Add RIO workspace source
echo "source ~/rio_ws/install/setup.bash" >> ~/.bashrc
```

> **Important**: For existing terminals, manually run these commands or restart your shell:
>
> ```bash
> # 2.4 Manual sourcing for existing terminals
> source /opt/ros/$ROS_DISTRO/setup.bash
> source ~/uros_ws/install/setup.bash
> source ~/rio_ws/install/setup.bash
> ```

### 3. Real Robot Launch

```bash
# 4.1 Launch real robot nodes
ros2 launch rio_bringup rio_real_robot.launch.py \
  use_sim_time:=false \
  agent_port:=8888
```

**Real Robot Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `use_sim_time` | Use simulation clock (must be false for real hardware) | `false` | `true`/`false` |
| `agent_port` | Micro-ROS agent UDP port | `8888` | Any available port number |

### 4. Simulation Launch

```bash
# 3.1 Launch Gazebo simulation
ros2 launch rio_simulation gazebo.launch.py \
  world:=house.world \
  use_sim_time:=true
```

**Simulation Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `world` | Gazebo world file | `empty.world` | `house.world`, `warehouse.world` |
| `use_sim_time` | Use simulation clock | `true` | `true`/`false` |

### 5. Mapping & Navigation

#### 5.1 Create Map

```bash
# 5.1.1 Launch SLAM mapping
ros2 launch rio_mapping mapping.launch.py \
  use_sim_time:=false
```
> **Note**: Refer mapping params in _rio_mapping/params/mapping_config.yaml_ for any modifications.

#### 5.2 Teleoperation Methods

##### 5.2.1 Joystick Teleop

```bash
# 5.2.1.1 Launch joystick teleop
ros2 launch rio_teleop teleop_joy.launch.py
```

> **Note**: Refer joystick params in _rio_teleop/params/joystick.yaml_ for any modifications.

##### 5.2.2 RQT Robot Steering GUI

```bash
# 5.2.2.1 Install RQT if not already installed
sudo apt install ros-$ROS_DISTRO-rqt-robot-steering

# 5.2.2.2 Launch RQT Robot Steering
ros2 run rqt_robot_steering rqt_robot_steering
```

> **Tip**: Ensure your robot's teleop topic is correctly configured.
> Typical topics include:
>
> - `/cmd_vel` for drive robot

#### 5.3 Save Map

```bash
# 5.3.1 Save created map
ros2 run nav2_map_server map_saver_cli -f <map_file_name>
```

This saves map files inside `rio_mapping/maps/` folder.

#### 5.4 Autonomous Navigation

```bash
# 5.4.1 Launch navigation
ros2 launch rio_navigation navigation.launch.py \
  map:=house.yaml \
  params_file:=nav2_real_params.yaml \
  use_sim_time:=false
```

**Navigation Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `map` | Map file for navigation | `house.yaml` | YAML map file name |
| `params_file` | Navigation parameters | `nav2_real_params.yaml` | YAML config file name, Available: `nav2_real_params.yaml`/ `nav2_sim_params.yaml` |
| `use_sim_time` | Use simulation clock | `false` | `true`/`false` |

### Visualization Tools

### 6. Visualization Tools

```bash
# 6.1 Launch RViz
ros2 launch rio_simulation rviz.launch.py \
  rviz_config:=default.rviz
```

**Visualization Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `rviz_config` | RViz config file | `default.rviz` | Any .rviz config |
