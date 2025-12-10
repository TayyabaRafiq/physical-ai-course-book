# Isaac Sim Setup and Configuration

## Overview

NVIDIA Isaac Sim is a GPU-accelerated robotics simulation platform built on NVIDIA Omniverse. It provides photorealistic rendering, accurate physics simulation, and seamless integration with Isaac ROS for AI-powered robot perception and navigation.

Isaac Sim transforms your development workflow by enabling:
- **GPU-Accelerated Physics**: Real-time physics simulation at scale
- **Photorealistic Rendering**: Ray-traced visualization for computer vision training
- **Synthetic Data Generation**: Generate labeled datasets for ML training
- **Digital Twin Workflows**: Test algorithms before deploying to hardware

## System Requirements

### Minimum Requirements

| Component | Requirement |
|-----------|-------------|
| **OS** | Ubuntu 20.04/22.04 or Windows 10/11 |
| **GPU** | NVIDIA RTX 2070 or higher (8GB+ VRAM) |
| **CPU** | Intel i7 or AMD Ryzen 7 |
| **RAM** | 32GB |
| **Storage** | 50GB free space (SSD recommended) |

### Recommended Requirements

| Component | Requirement |
|-----------|-------------|
| **GPU** | NVIDIA RTX 3090/4090 or A6000 (24GB VRAM) |
| **CPU** | Intel i9 or AMD Ryzen 9 |
| **RAM** | 64GB |
| **Storage** | 100GB NVMe SSD |

## Installation

### 1. Install NVIDIA Drivers

```bash
# Ubuntu - Install latest NVIDIA driver
sudo apt update
sudo apt install nvidia-driver-535

# Verify installation
nvidia-smi
```

Expected output should show driver version 535+ and CUDA 12.0+.

### 2. Install Omniverse Launcher

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**Windows**: Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/download/)

### 3. Install Isaac Sim via Launcher

1. Open Omniverse Launcher
2. Navigate to **Exchange** tab
3. Search for **Isaac Sim**
4. Click **Install** (version 2023.1.0 or later)
5. Wait for installation (10-20 minutes)

### 4. Verify Installation

Launch Isaac Sim from the Launcher:

```bash
# Or launch via command line (Linux)
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

You should see the Isaac Sim GUI with the default scene.

## Workspace Configuration

### Create Isaac ROS Workspace

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build workspace
cd ~/isaac_ros_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Configure Environment Variables

Add to `~/.bashrc`:

```bash
# Isaac Sim paths
export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-2023.1.0
export ISAAC_ROS_WS=~/isaac_ros_ws

# Source ROS 2
source /opt/ros/humble/setup.bash
source $ISAAC_ROS_WS/install/setup.bash

# Python path for Isaac Sim
export PYTHONPATH=$PYTHONPATH:$ISAAC_SIM_PATH/exts/omni.isaac.kit/pip_prebundle
```

Apply changes:

```bash
source ~/.bashrc
```

## Isaac Sim Interface Overview

### Main UI Components

```
┌─────────────────────────────────────────────────┐
│  Menu Bar: File | Create | Window | Help        │
├─────────┬───────────────────────────┬───────────┤
│         │                           │           │
│ Stage   │   Viewport (3D Scene)     │ Property  │
│ Tree    │                           │ Panel     │
│         │                           │           │
│ - World │                           │ Transform │
│   - Robot│                          │ Physics   │
│   - Env │                           │ Materials │
│         │                           │           │
├─────────┴───────────────────────────┴───────────┤
│  Content Browser: Assets and Extensions         │
└─────────────────────────────────────────────────┘
```

### Key Panels

1. **Stage Tree**: Hierarchical scene graph
2. **Viewport**: 3D visualization with camera controls
3. **Property Panel**: Object properties and physics parameters
4. **Content Browser**: Asset library and file browser
5. **Console**: Python scripting and logging

## Loading Robot Models

### Import URDF

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load robot URDF
robot_path = "/path/to/robot.urdf"
add_reference_to_stage(
    usd_path=robot_path,
    prim_path="/World/Robot"
)
```

### Import USD (Omniverse Format)

```python
from pxr import Usd, UsdGeom

# Open USD stage
stage = Usd.Stage.Open("/path/to/robot.usd")

# Add to scene
prim = stage.GetPrimAtPath("/World/Robot")
```

### Verify Robot Import

1. Check **Stage Tree** for robot hierarchy
2. Verify joints in **Property Panel**
3. Test physics by clicking **Play** button

## Physics Configuration

### Enable Physics

```python
import omni.physx as physx
from pxr import PhysxSchema

# Get physics scene
physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World"))

# Configure physics parameters
physx_scene_api.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
physx_scene_api.CreateGravityMagnitudeAttr().Set(9.81)
physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
```

### Configure Joints

```python
from omni.isaac.core.utils.prims import get_prim_at_path

# Get joint prim
joint = get_prim_at_path("/World/Robot/joint_1")

# Set joint properties
joint.GetAttribute("drive:angular:physics:damping").Set(100.0)
joint.GetAttribute("drive:angular:physics:stiffness").Set(10000.0)
joint.GetAttribute("physics:upperLimit").Set(3.14)
joint.GetAttribute("physics:lowerLimit").Set(-3.14)
```

## Sensor Setup

### Add Camera

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera

# Create camera
camera = Camera(
    prim_path="/World/Robot/camera",
    position=[0.5, 0.0, 0.3],
    orientation=[0, 0, 0, 1],
    resolution=(1280, 720),
    frequency=30
)

# Initialize camera
camera.initialize()

# Get RGB image
rgb_data = camera.get_rgba()
```

### Add Lidar

```python
from omni.isaac.range_sensor import _range_sensor

# Create lidar
result, lidar = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path="/World/Robot/lidar",
    parent="/World/Robot",
    config="Velodyne_VLP16"
)

# Configure scan pattern
lidar.set_rotation_frequency(10.0)  # 10 Hz
lidar.set_horizontal_fov(360.0)
lidar.set_horizontal_resolution(0.2)
```

### Add IMU

```python
from omni.isaac.sensor import IMUSensor

# Create IMU
imu = IMUSensor(
    prim_path="/World/Robot/imu",
    translation=[0.0, 0.0, 0.1],
    orientation=[0, 0, 0, 1]
)

# Get IMU data
linear_accel = imu.get_linear_acceleration()
angular_vel = imu.get_angular_velocity()
```

## ROS 2 Bridge Configuration

### Enable ROS 2 Bridge Extension

1. **Window → Extensions**
2. Search for **ROS2 Bridge**
3. Click **Enable**

### Configure ROS Topics

```python
import omni.graph.core as og

# Create ROS action graph
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ("PublishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
        ],
    },
)
```

### Test ROS Connection

```bash
# In terminal 1: Launch Isaac Sim with ROS bridge
# (Run from Omniverse)

# In terminal 2: Check ROS topics
ros2 topic list

# Expected output:
# /clock
# /tf
# /tf_static
# /odom
# /joint_states
```

## Standalone Python Scripts

### Run Isaac Sim Headless

```python
from omni.isaac.kit import SimulationApp

# Launch headless simulation
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot

# Create world
world = World()

# Add robot
robot = world.scene.add(Robot(prim_path="/World/Robot", name="my_robot"))

# Run simulation
world.reset()
for _ in range(1000):
    world.step(render=False)

# Cleanup
simulation_app.close()
```

### Batch Simulation

```bash
# Run headless simulation
~/.local/share/ov/pkg/isaac_sim-*/python.sh my_simulation.py
```

## Performance Optimization

### GPU Acceleration

Enable GPU physics for maximum performance:

```python
# Enable GPU dynamics
physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)

# Use GPU pipeline
physx_scene_api.CreateBroadphaseTypeAttr().Set("GPU")
```

### Reduce Rendering Load

```python
# Disable expensive rendering features
import carb.settings
settings = carb.settings.get_settings()

settings.set("/rtx/raytracing/enabled", False)
settings.set("/rtx/indirectDiffuse/enabled", False)
settings.set("/rtx/reflections/enabled", False)
```

### Optimize Physics Settings

```python
# Reduce simulation frequency for faster-than-real-time
world.set_simulation_dt(physics_dt=1.0/60.0, rendering_dt=1.0/30.0)
```

## Common Issues and Solutions

### Issue: Isaac Sim won't launch

**Solution**: Check GPU drivers
```bash
nvidia-smi
# Should show CUDA 12.0+ and driver 535+
```

### Issue: Robot falls through floor

**Solution**: Add collision meshes
```python
# Ensure collision meshes are defined
from omni.isaac.core.utils.prims import create_prim

ground = create_prim("/World/Ground", "Cube", scale=[10, 10, 0.1])
# Add physics collision
```

### Issue: Slow performance

**Solution**: Enable GPU physics and reduce rendering
```python
physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
settings.set("/rtx/raytracing/enabled", False)
```

## Best Practices

1. **Use USD Format**: Convert URDF to USD for better performance
2. **Enable GPU Physics**: Always use GPU acceleration when available
3. **Organize Scene Hierarchy**: Keep prims organized in logical groups
4. **Version Control USD Files**: Track changes to robot models
5. **Profile Performance**: Use built-in profiler (Window → Profiler)

## Integration with VLA Pipeline

Isaac Sim can serve as the simulation backend for the VLA ActionService:

```python
class IsaacActionService(IActionService):
    def __init__(self):
        self.world = World()
        self.robot = self.world.scene.add(Robot(...))

    async def execute_action(self, action, feedback_callback):
        # Execute in Isaac Sim
        # Convert action to Isaac Sim commands
        # Provide realistic physics feedback
        pass
```

This enables testing the VLA pipeline with accurate physics before deploying to hardware.

## Next Steps

- **[Chapter 2: Isaac ROS VSLAM](./02-isaac-ros-vslam)**: Visual-inertial odometry and mapping
- **[Chapter 3: Nav2 Integration](./03-nav2-integration)**: Autonomous navigation stack
- **[Chapter 4: Autonomous Navigation](./04-autonomous-navigation)**: Complete navigation pipeline

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Omniverse USD Documentation](https://docs.omniverse.nvidia.com/usd/latest/index.html)
- [PhysX Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.1.3/index.html)
