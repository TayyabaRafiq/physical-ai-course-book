# ROS 2 Fundamentals

## Overview

ROS 2 (Robot Operating System 2) serves as the **nervous system** for modern robots, providing a distributed communication framework that allows different robot components to work together seamlessly. Just as the human nervous system coordinates muscles, sensors, and the brain, ROS 2 coordinates robot actuators, sensors, and control algorithms.

:::info Note on VLA MVP
ROS 2 is **optional** for the VLA MVP. The core system uses Pure Python execution and can be extended to ROS 2 when ready for physical robot deployment.
:::

## What is ROS 2?

ROS 2 is a flexible middleware framework for robot software development that provides:

- **Communication Layer**: Publish-subscribe messaging, services, and actions
- **Hardware Abstraction**: Unified interfaces for sensors and actuators
- **Distributed Architecture**: Multi-process, multi-machine coordination
- **Tool Ecosystem**: Visualization, logging, debugging, and simulation

### Key Differences from ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Communication** | Custom TCPROS | DDS (Data Distribution Service) |
| **Real-Time** | Limited | Real-time capable |
| **Security** | None | Built-in encryption & authentication |
| **Multi-Robot** | Complex | Native support |
| **Platforms** | Linux only | Linux, Windows, macOS |

## Core Concepts

### 1. Nodes

Nodes are individual processes that perform computation:

```bash
# Create a simple ROS 2 node
ros2 run demo_nodes_py talker
```

**Characteristics**:
- Single-purpose processes (e.g., camera driver, path planner)
- Communicate via topics, services, or actions
- Independently deployable and testable

### 2. Topics (Publish-Subscribe)

Topics enable asynchronous, many-to-many communication:

```bash
# List active topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /chatter

# Publish to a topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

**Use Cases**:
- Sensor data streams (camera images, lidar scans)
- Robot state broadcasts (odometry, joint states)
- Command channels (velocity commands)

### 3. Services (Request-Reply)

Services provide synchronous, one-to-one communication:

```bash
# List available services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Use Cases**:
- Configuration changes
- One-time queries (get map, compute IK)
- Stateless operations

### 4. Actions (Goal-Oriented Tasks)

Actions support long-running, preemptable tasks with feedback:

```bash
# Send an action goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}}}}"
```

**Components**:
- **Goal**: Task specification
- **Feedback**: Progress updates
- **Result**: Final outcome

**Use Cases**:
- Navigation to target
- Pick-and-place operations
- Trajectory execution

## ROS 2 Architecture

```
┌─────────────────────────────────────────────┐
│  Application Layer                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │  Node A  │  │  Node B  │  │  Node C  │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  │
└───────┼─────────────┼─────────────┼─────────┘
        │             │             │
┌───────┼─────────────┼─────────────┼─────────┐
│       │   ROS 2 Client Libraries  │         │
│       │   (rclcpp, rclpy)          │         │
└───────┼─────────────┼─────────────┼─────────┘
        │             │             │
┌───────┼─────────────┼─────────────┼─────────┐
│       │        ROS Middleware (rmw)         │
│       │        (DDS Implementation)         │
└───────┼─────────────┼─────────────┼─────────┘
        │             │             │
        └─────────────┴─────────────┘
           Network Transport Layer
```

## Installation

### Ubuntu 22.04 (Humble)

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop

# Source environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Windows 10/11

```powershell
# Download installer from https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
# Run installer and add to PATH
# Open ROS 2 shell via Start Menu
```

### macOS

```bash
# Install via Homebrew
brew install ros2
```

## Verification

```bash
# Check installation
ros2 --version

# Run demo talker
ros2 run demo_nodes_cpp talker

# In another terminal, run listener
ros2 run demo_nodes_cpp listener
```

## Workspace Setup

### Creating a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

### Adding Packages

```bash
cd ~/ros2_ws/src

# Create a new package
ros2 pkg create --build-type ament_python my_robot_pkg \
  --dependencies rclpy std_msgs

# Build
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
```

## Quality of Service (QoS)

ROS 2 uses DDS QoS policies to control communication behavior:

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | RELIABLE, BEST_EFFORT | Sensor data vs. commands |
| **Durability** | VOLATILE, TRANSIENT_LOCAL | Ephemeral vs. latched |
| **History** | KEEP_LAST(n), KEEP_ALL | Buffer management |
| **Lifespan** | Duration | Message expiration |

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'topic', qos)
```

## Common Tools

### Introspection

```bash
# Node information
ros2 node info /my_node

# Topic bandwidth
ros2 topic bw /camera/image_raw

# Service type
ros2 service type /add_two_ints

# Parameter list
ros2 param list
```

### Visualization

```bash
# rqt GUI
rqt

# View TF tree
ros2 run tf2_tools view_frames

# RViz 2 (3D visualization)
rviz2
```

### Recording and Playback

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image /odom

# Playback
ros2 bag play my_bag.db3
```

## Best Practices

1. **Single Responsibility**: Each node should do one thing well
2. **Namespace Usage**: Organize nodes and topics in namespaces
3. **Parameter Configuration**: Use YAML files for configuration
4. **Launch Files**: Automate multi-node startup with launch files
5. **QoS Tuning**: Match QoS policies to communication patterns

## Integration with VLA Pipeline

While the VLA MVP uses Pure Python execution, ROS 2 can be added as an optional backend:

```python
from src.vla_core.contracts.action_service import IActionService

# Pure Python (default)
service = ActionService()

# ROS 2 backend (optional)
service = RosActionService()

# Both implement same interface
await service.execute_action(action, feedback_callback)
```

This allows transparent switching between simulated and real robot execution.

## Next Steps

- **[Chapter 2: Python-ROS Bridge (rclpy)](./02-rclpy-bridge)**: Learn to write ROS 2 nodes in Python
- **[Chapter 3: URDF for Humanoids](./03-urdf-humanoids)**: Model robot kinematics and dynamics

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Principles](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
