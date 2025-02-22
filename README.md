<div align="center">
    <img src="https://img.shields.io/github/stars/botforge-robotics/rio_ros2?style=social&logo=github" alt="Stars">&nbsp;
    <img src="https://img.shields.io/github/forks/botforge-robotics/rio_ros2?style=social&logo=github" alt="Forks">&nbsp;
    <img src="https://img.shields.io/github/issues/botforge-robotics/rio_ros2" alt="Issues">&nbsp;
    <img src="https://img.shields.io/github/repo-size/botforge-robotics/rio_ros2" alt="Repo Size">&nbsp;
    <img src="https://img.shields.io/github/license/botforge-robotics/rio_ros2?color=mit" alt="MIT License">
</div>

<!-- Start of Selection -->
<h2 align="center">Welcome to the RIO Project!</h2>
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

### Hardware Requirements

- Assembled RIO Robot with Control PCB
- Smartphone with RIO App.
- PC with ROS2 Installed

## 🚀 Getting Started

### Environment Setup

```bash
# Source ROS installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Set up Micro-ROS workspace
mkdir -p ~/uros_ws/src
cd ~/uros_ws/src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
cd ~/uros_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash

# Set up RIO workspace
mkdir -p ~/rio_ws/src
cd ~/rio_ws/src
git clone https://github.com/botforge-robotics/rio_ros2.git
cd ~/rio_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Terminal Configuration

Add these to your `~/.bashrc` for automatic sourcing in new terminals:

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source ~/uros_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/rio_ws/install/setup.bash" >> ~/.bashrc
```

For existing terminals, manually run these commands or restart your shell:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/uros_ws/install/setup.bash
source ~/rio_ws/install/setup.bash
```

### Simulation Launch

```bash
ros2 launch rio_simulation gazebo.launch.py \
  world:=house.world \
  use_sim_time:=true
```

**Simulation Parameters**:
| Parameter | Description | Default Value | Options |
|-----------------|-------------------------------|----------------|-----------------------|
| `world` | Gazebo world file | `empty.world` | `house.world`, `warehouse.world` |
| `use_sim_time` | Use simulation clock | `true` | `true`/`false` |

### Real Robot Launch

```bash
ros2 launch rio_bringup rio_real_robot.launch.py \
  use_sim_time:=false \
  agent_port:=8888
```

**Real Robot Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `use_sim_time` | Use simulation clock (must be false for real hardware) | `false` | `true`/`false` |
| `agent_port` | Micro-ROS agent UDP port | `8888` | Any available port number |

> **Hardware Note**: Ensure physical robot is powered on and connected via WiFi after launching. Click on RST button on pcb for any connection issue.

### Mapping & Navigation

**1. Create Map**:

```bash
ros2 launch rio_mapping mapping.launch.py \
  use_sim_time:=false
```

**Mapping Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `use_sim_time` | Use simulation clock | `false` | `true`/`false` |

**2. Save Map**:

```bash
ros2 run nav2_map_server map_saver_cli -f <filename>
```
This saves map files inside `rio_mapping/maps/` folder.

**3. Navigation**:

```bash
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

```bash
ros2 launch rio_simulation rviz.launch.py \
  rviz_config:=default.rviz
```

**Visualization Parameters**:
| Parameter | Description | Default Value | Options |
|-----------|-------------|---------------|---------|
| `rviz_config` | RViz config file | `default.rviz` | Any .rviz config |
### Teleoperation Joystic Control

```bash
ros2 launch rio_teleop teleop_joy.launch.py
```
> **Note**: Refer joystic params in *rio_teleop/params/joystick.yaml* for any modifications.