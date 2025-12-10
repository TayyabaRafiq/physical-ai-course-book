# URDF for Humanoid Robots

## Overview

**URDF (Unified Robot Description Format)** is an XML-based language for modeling robot kinematics, dynamics, and visual appearance. For humanoid robots, URDF provides the blueprint that defines joint hierarchies, link geometry, mass properties, and sensor placements.

## Why URDF for Humanoids?

Humanoid robots present unique modeling challenges:

| Challenge | URDF Solution |
|-----------|---------------|
| **Complex Kinematics** | Hierarchical link-joint trees |
| **High DOF** | 30+ joints (arms, legs, torso, head) |
| **Dynamic Balance** | Accurate mass/inertia properties |
| **Sensor Integration** | Camera, IMU, force/torque sensors |
| **Visualization** | Meshes for simulation rendering |

## URDF Structure

### Minimal Humanoid Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.6" iyz="0.0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Hip Joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="pelvis"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <!-- Pelvis Link -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.35 0.4 0.2"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0"
               iyy="0.4" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

</robot>
```

## Core Components

### 1. Links

Links represent rigid bodies with physical properties:

```xml
<link name="upper_arm">
  <!-- Visual (rendering) -->
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>

  <!-- Collision (physics) -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>

  <!-- Inertial (dynamics) -->
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="2.0"/>
    <inertia ixx="0.015" ixy="0" ixz="0"
             iyy="0.015" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

**Geometry Types**:
- `<box>`: Cuboid (`size="x y z"`)
- `<cylinder>`: Cylinder (`length`, `radius`)
- `<sphere>`: Sphere (`radius`)
- `<mesh>`: 3D model (`filename`, `scale`)

### 2. Joints

Joints connect links and define motion constraints:

```xml
<!-- Revolute Joint (1 DOF rotation) -->
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.2 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Pitch axis -->
  <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  <dynamics damping="0.7" friction="0.1"/>
</joint>
```

**Joint Types**:
- `revolute`: Hinge joint with limits
- `continuous`: Hinge without limits (wheels)
- `prismatic`: Sliding joint (telescoping)
- `fixed`: Rigid connection
- `floating`: 6 DOF (base link)
- `planar`: 2D motion (not common)

**Axis Convention**:
- X: Forward/backward
- Y: Left/right
- Z: Up/down

## Humanoid Kinematic Tree

Typical humanoid joint hierarchy:

```
                    torso
                      |
      ┌───────────────┼───────────────┐
      |               |               |
    head           pelvis         shoulders
                      |               |
              ┌───────┴───────┐   ┌───┴───┐
              |               |   |       |
          left_hip      right_hip left  right
              |               |   arm    arm
          left_knee      right_knee |     |
              |               |   elbow elbow
         left_ankle    right_ankle  |     |
              |               |   wrist wrist
         left_foot      right_foot  |     |
                                 gripper gripper
```

## Full Humanoid Example

```xml
<?xml version="1.0"?>
<robot name="atlas_humanoid">

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <mesh filename="package://atlas_description/meshes/torso.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.7"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="1.0" ixy="0" ixz="0"
               iyy="1.2" iyz="0" izz="0.8"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_pitch" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7" upper="0.7" effort="20" velocity="2.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm Chain -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.25 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.8" upper="2.8" effort="87" velocity="2.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="50" velocity="2.0"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Repeat for right arm, legs, etc. -->

</robot>
```

## Xacro: Programmable URDF

Xacro extends URDF with macros to reduce repetition:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Properties (constants) -->
  <xacro:property name="arm_length" value="0.35"/>
  <xacro:property name="arm_radius" value="0.05"/>
  <xacro:property name="arm_mass" value="2.5"/>

  <!-- Macro for arm link -->
  <xacro:macro name="arm_link" params="prefix length radius mass">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${mass*length*length/12}" ixy="0" ixz="0"
                 iyy="${mass*length*length/12}" iyz="0"
                 izz="${mass*radius*radius/2}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Instantiate left and right arms -->
  <xacro:arm_link prefix="left" length="${arm_length}"
                  radius="${arm_radius}" mass="${arm_mass}"/>
  <xacro:arm_link prefix="right" length="${arm_length}"
                  radius="${arm_radius}" mass="${arm_mass}"/>

</robot>
```

**Convert Xacro to URDF**:
```bash
xacro humanoid.xacro > humanoid.urdf
```

## Sensors in URDF

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>head_camera</cameraName>
      <imageTopicName>/camera/image_raw</imageTopicName>
    </plugin>
  </sensor>
</gazebo>
```

### IMU

```xml
<link name="imu_link"/>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <update_rate>100.0</update_rate>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Calculating Inertia

For basic shapes:

**Box** (dimensions: x, y, z):
```
Ixx = (1/12) * m * (y² + z²)
Iyy = (1/12) * m * (x² + z²)
Izz = (1/12) * m * (x² + y²)
```

**Cylinder** (radius: r, length: h, axis: z):
```
Ixx = Iyy = (1/12) * m * (3*r² + h²)
Izz = (1/2) * m * r²
```

**Sphere** (radius: r):
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

## Visualization and Validation

### View in RViz

```bash
# Install URDF tools
sudo apt install ros-humble-urdf-tutorial

# Launch URDF visualization
ros2 launch urdf_tutorial display.launch.py model:=humanoid.urdf
```

### Check URDF Validity

```bash
# Validate syntax
check_urdf humanoid.urdf

# View kinematic tree
urdf_to_graphiz humanoid.urdf
evince humanoid.pdf
```

### Publish Robot State

```python
from sensor_msgs.msg import JointState

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_shoulder_pitch', 'left_elbow', 'right_shoulder_pitch', 'right_elbow']
        msg.position = [0.5, 1.0, -0.5, 1.2]
        msg.velocity = []
        msg.effort = []
        self.publisher.publish(msg)
```

## Integration with Gazebo

URDF files are used by Gazebo for simulation:

```bash
# Spawn robot in Gazebo
ros2 run gazebo_ros spawn_entity.py -entity humanoid -file humanoid.urdf
```

See **[Module 2: Digital Twin Simulation](../02-digital-twin-simulation/01-simulation-engines)** for details.

## Best Practices

1. **Use Xacro**: Avoid repetition with macros and properties
2. **Accurate Inertia**: Essential for realistic dynamics
3. **Mesh Simplification**: Use low-poly collision meshes
4. **Origin Placement**: Set link origins at joint centers
5. **Joint Limits**: Match physical robot constraints
6. **Namespace Conventions**: Prefix links consistently (`left_`, `right_`)

## Common Issues

| Problem | Solution |
|---------|----------|
| **Robot falls through ground** | Check collision geometry and mass |
| **Joints unstable** | Add damping and friction |
| **Visual misalignment** | Verify `<origin>` in visual/collision |
| **Parsing errors** | Validate with `check_urdf` |

## Next Steps

- **[Module 2: Digital Twin Simulation](../02-digital-twin-simulation/01-simulation-engines)**: Simulate URDF models
- **[Module 3: Isaac Sim](../03-isaac-robot-brain/01-isaac-sim-setup)**: GPU-accelerated physics

## Additional Resources

- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Gazebo URDF Extensions](http://classic.gazebosim.org/tutorials?tut=ros_urdf)
- [SolidWorks to URDF](http://wiki.ros.org/sw_urdf_exporter)
