# Simulation Engines for Digital Twins

## Overview

**Digital twin** technology creates virtual replicas of physical robots, enabling safe testing, validation, and training before deployment to real hardware. Simulation engines provide the physics, rendering, and sensor modeling needed to create high-fidelity digital twins.

## What is a Digital Twin?

A digital twin is a virtual representation that mirrors:

| Component | Physical | Digital Twin |
|-----------|----------|--------------|
| **Structure** | Robot chassis, links, joints | URDF/SDF model |
| **Physics** | Mass, inertia, friction | Physics engine simulation |
| **Sensors** | Cameras, lidar, IMU | Simulated sensor data |
| **Actuators** | Motors, servos | Torque/velocity commands |
| **Environment** | Warehouse, lab | Virtual world |

## Why Use Simulation?

### Advantages

1. **Safety**: Test dangerous scenarios without risk
2. **Cost**: No hardware damage or replacement costs
3. **Speed**: Parallel testing across multiple scenarios
4. **Iteration**: Rapid prototyping and algorithm development
5. **Accessibility**: Develop without physical robot access

### Limitations

1. **Reality Gap**: Simulation ≠ reality (friction, sensor noise, deformation)
2. **Computation**: High-fidelity simulation requires GPU resources
3. **Model Accuracy**: Results depend on accurate modeling
4. **Validation**: Must verify on real hardware eventually

## Simulation Engine Landscape

### Gazebo Classic (Deprecated)

- **Status**: Replaced by Gazebo (formerly Ignition)
- **Use**: Legacy ROS 1 projects
- **Physics**: ODE, Bullet, Simbody, DART
- **Not Recommended**: Use Gazebo (Ignition) instead

### Gazebo (Ignition → Gazebo)

- **Current Version**: Gazebo Harmonic (2024)
- **Physics**: DART by default
- **Rendering**: Ogre 2.x (PBR materials)
- **Integration**: Native ROS 2 support
- **Best For**: General robotics simulation

### PyBullet

- **Type**: Python physics library
- **Physics**: Bullet3
- **Rendering**: OpenGL (basic)
- **Best For**: Reinforcement learning, research prototypes
- **Advantage**: Pure Python, no ROS required

### MuJoCo

- **Owner**: DeepMind (open source since 2022)
- **Physics**: Custom (very fast, accurate)
- **Rendering**: OpenGL
- **Best For**: Contact-rich manipulation, RL
- **Advantage**: Fast dynamics, excellent for contact

### Isaac Sim (NVIDIA)

- **Platform**: Omniverse
- **Physics**: PhysX 5 (GPU-accelerated)
- **Rendering**: RTX ray tracing
- **Best For**: Large-scale sim, AI training, photorealistic sensors
- **Covered In**: [Module 3: Isaac Sim](../03-isaac-robot-brain/01-isaac-sim-setup)

### Unity Robotics

- **Platform**: Unity game engine
- **Physics**: PhysX
- **Rendering**: HDRP (high-fidelity)
- **Integration**: ROS TCP Connector
- **Best For**: Human-robot interaction, AR/VR

### Unreal Engine + NVIDIA Isaac Sim

- **Rendering**: Unreal Engine 5 (Nanite, Lumen)
- **Physics**: Chaos (Unreal) or PhysX (Isaac)
- **Best For**: Photorealistic visualization

## Comparison Matrix

| Engine | Physics | Rendering | ROS 2 | GPU Accel | Learning Curve | License |
|--------|---------|-----------|-------|-----------|----------------|---------|
| **Gazebo** | DART | Ogre2 | Native | Partial | Medium | Apache 2.0 |
| **PyBullet** | Bullet3 | OpenGL | Manual | No | Low | Zlib |
| **MuJoCo** | Custom | OpenGL | Manual | Partial | Medium | Apache 2.0 |
| **Isaac Sim** | PhysX5 | RTX | Native | Full | High | Proprietary |
| **Unity** | PhysX | HDRP | Plugin | Yes | Medium | Proprietary |

## Gazebo Architecture

```
┌─────────────────────────────────────────────┐
│  Gazebo Server (gz sim)                     │
│  ┌─────────────┐    ┌─────────────┐        │
│  │   Physics   │───▶│   Sensors   │        │
│  │   Engine    │    │  (Camera,   │        │
│  │   (DART)    │    │   Lidar)    │        │
│  └──────┬──────┘    └─────┬───────┘        │
│         │                 │                 │
│    ┌────▼────────────────▼────┐            │
│    │  Scene Graph (Entities) │            │
│    └────────────┬────────────┘            │
└─────────────────┼──────────────────────────┘
                  │ ROS 2 Bridge
┌─────────────────▼──────────────────────────┐
│  ROS 2 Nodes                               │
│  ┌─────────┐  ┌─────────┐  ┌──────────┐   │
│  │ Control │  │ Sensors │  │Navigation│   │
│  └─────────┘  └─────────┘  └──────────┘   │
└────────────────────────────────────────────┘
```

## Installing Simulation Engines

### Gazebo Harmonic (Recommended)

```bash
# Ubuntu 22.04
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo
sudo apt-get update
sudo apt-get install gz-harmonic

# Verify
gz sim --version
```

### PyBullet

```bash
pip install pybullet
```

```python
import pybullet as p
import pybullet_data

# Start simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot = p.loadURDF("humanoid.urdf", [0, 0, 1])

# Simulate
for _ in range(10000):
    p.stepSimulation()
```

### MuJoCo

```bash
pip install mujoco
```

```python
import mujoco
import mujoco.viewer

# Load model
model = mujoco.MjModel.from_xml_path('humanoid.xml')
data = mujoco.MjData(model)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

## World Modeling

### SDF (Simulation Description Format)

SDF extends URDF with simulation-specific features:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_world">

    <!-- Physics Engine Configuration -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Sun Light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include Robot Model -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Add Objects -->
    <model name="box">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Physics Configuration

### Timestep Selection

```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>  <!-- 1ms for stable humanoid simulation -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

**Guidelines**:
- Humanoids: 0.001s (1000 Hz)
- Mobile robots: 0.01s (100 Hz)
- Manipulators: 0.002s (500 Hz)

### Solver Parameters

```xml
<dart>
  <solver>
    <solver_type>dantzig</solver_type>
    <iterations>50</iterations>
  </solver>
  <collision_detector>bullet</collision_detector>
</dart>
```

## Performance Optimization

### 1. Collision Mesh Simplification

```xml
<!-- High-detail visual mesh -->
<visual>
  <geometry>
    <mesh><uri>model://humanoid/meshes/torso_highpoly.dae</uri></mesh>
  </geometry>
</visual>

<!-- Simplified collision mesh -->
<collision>
  <geometry>
    <mesh><uri>model://humanoid/meshes/torso_collision.dae</uri></mesh>
  </geometry>
</collision>
```

### 2. Parallel Simulation

```bash
# Run multiple Gazebo instances
for i in {1..4}; do
  gz sim -s -r world_$i.sdf &
done
```

### 3. Headless Mode

```bash
# No GUI (faster)
gz sim -s world.sdf
```

## Integration with VLA Pipeline

The VLA MVP uses **Pure Python** execution by default, but can integrate with simulation:

```python
from src.vla_core.contracts.action_service import IActionService

# Option 1: Pure Python (default)
service = ActionService()

# Option 2: Gazebo backend
service = GazeboActionService(world_name='humanoid_world')

# Same interface
await service.execute_action(action, feedback_callback)
```

## Next Steps

- **[Chapter 2: Gazebo Physics](./02-gazebo-physics)**: Deep dive into physics simulation
- **[Chapter 3: Unity for HRI](./03-unity-hri)**: High-fidelity visualization
- **[Chapter 4: Simulated Sensors](./04-simulated-sensors)**: Camera, lidar, IMU modeling

## Additional Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [PyBullet Quickstart](https://pybullet.org/wordpress/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [SDF Specification](http://sdformat.org/)
