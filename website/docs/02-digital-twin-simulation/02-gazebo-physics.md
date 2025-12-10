# Gazebo Physics Simulation

## Overview

Physics simulation is the heart of digital twins, computing how robots interact with their environment through forces, contacts, and constraints. Gazebo uses **DART (Dynamic Animation and Robotics Toolkit)** by default to provide accurate, stable physics for complex humanoid robots.

## Physics Engines in Gazebo

Gazebo supports multiple physics backends:

| Engine | Strengths | Weaknesses | Best For |
|--------|-----------|------------|----------|
| **DART** | Stable contacts, fast | Learning curve | Humanoids, manipulation |
| **Bullet** | Fast, widely used | Contact instability | Mobile robots, gaming |
| **ODE** | Legacy support | Slow, inaccurate | Old projects |
| **Simbody** | Biomechanics | Complex setup | Human modeling |

**Recommendation**: Use DART for humanoid robots.

## Configuring DART Physics

### World-Level Configuration

```xml
<world name="humanoid_world">
  <physics name="dart_physics" type="dart">
    <!-- Simulation Timestep -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>

    <!-- DART-Specific Settings -->
    <dart>
      <solver>
        <solver_type>dantzig</solver_type>
        <iterations>50</iterations>
      </solver>
      <collision_detector>bullet</collision_detector>
    </dart>
  </physics>

  <!-- Gravity -->
  <gravity>0 0 -9.81</gravity>
</world>
```

### Model-Level Overrides

```xml
<model name="humanoid">
  <link name="torso">
    <collision name="torso_collision">
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
        <contact>
          <ode>
            <kp>1e6</kp>      <!-- Contact stiffness -->
            <kd>100</kd>      <!-- Contact damping -->
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
      </surface>
    </collision>
  </link>
</model>
```

## Contact Dynamics

### Friction Models

```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>     <!-- Primary friction coefficient -->
      <mu2>0.6</mu2>   <!-- Secondary (orthogonal) friction -->
      <slip1>0.01</slip1>
      <slip2>0.01</slip2>
    </ode>
  </friction>
</surface>
```

**Typical Values**:
- Rubber on concrete: μ = 0.7-1.0
- Metal on metal: μ = 0.15-0.25
- Gripper on object: μ = 0.8-1.2

### Contact Constraints

```xml
<contact>
  <ode>
    <kp>100000.0</kp>    <!-- Spring constant (N/m) -->
    <kd>100.0</kd>       <!-- Damping coefficient (N·s/m) -->
    <max_vel>0.01</max_vel>  <!-- Max penetration correction velocity -->
    <min_depth>0.001</min_depth>
  </ode>
</contact>
```

**Tuning Guidelines**:
- Higher `kp`: Stiffer contacts (less penetration)
- Higher `kd`: More damping (less bounce)
- Lower `max_vel`: Smoother contact resolution

## Joint Dynamics

### Revolute Joint with Damping

```xml
<joint name="knee_joint" type="revolute">
  <parent>upper_leg</parent>
  <child>lower_leg</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>0</lower>
      <upper>2.5</upper>
      <effort>150</effort>      <!-- Max torque (N·m) -->
      <velocity>5.0</velocity>  <!-- Max angular velocity (rad/s) -->
    </limit>
    <dynamics>
      <damping>5.0</damping>    <!-- Joint damping (N·m·s/rad) -->
      <friction>0.5</friction>  <!-- Coulomb friction (N·m) -->
      <spring_reference>0.0</spring_reference>
      <spring_stiffness>0.0</spring_stiffness>
    </dynamics>
  </axis>
</joint>
```

### PID Joint Control

```xml
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
  <joint_name>left_shoulder_pitch</joint_name>
  <p_gain>1000</p_gain>
  <i_gain>50</i_gain>
  <d_gain>10</d_gain>
  <i_max>100</i_max>
  <i_min>-100</i_min>
  <cmd_max>87</cmd_max>      <!-- Effort limit -->
  <cmd_min>-87</cmd_min>
</plugin>
```

**PID Tuning**:
1. Start with P-only: Set `p_gain` until oscillation
2. Reduce P, add D: `d_gain = 0.1 * p_gain`
3. Add I if needed: `i_gain = 0.01 * p_gain`

## Humanoid Balance and Stability

### Center of Mass Tracking

```python
import gz.sim
import gz.math

# Get link pose
torso_pose = link.WorldPose()
com = torso_pose.Pos()

# Check if CoM is within support polygon
support_polygon = calculate_foot_support_polygon()
is_stable = point_in_polygon(com.X(), com.Y(), support_polygon)
```

### Zero Moment Point (ZMP)

The ZMP must stay within the support polygon for stable walking:

```python
def calculate_zmp(links, gravity=-9.81):
    """Calculate Zero Moment Point."""
    total_mass = 0
    weighted_x = 0
    weighted_y = 0

    for link in links:
        mass = link.Inertial().Mass()
        pos = link.WorldPose().Pos()
        accel = link.WorldLinearAccel()

        total_mass += mass
        weighted_x += mass * (pos.X() + accel.X() / gravity)
        weighted_y += mass * (pos.Y() + accel.Y() / gravity)

    zmp_x = weighted_x / total_mass
    zmp_y = weighted_y / total_mass
    return (zmp_x, zmp_y)
```

## Simulation Performance

### Timestep Guidelines

| Robot Type | Max Step Size | Update Rate | Real-Time Factor |
|------------|---------------|-------------|------------------|
| **Humanoid** | 0.001s (1ms) | 1000 Hz | 1.0 |
| **Quadruped** | 0.002s | 500 Hz | 1.0 |
| **Manipulator** | 0.002s | 500 Hz | 1.0 |
| **Mobile Robot** | 0.01s | 100 Hz | 1.0-2.0 |

### Measuring Performance

```bash
# Monitor real-time factor
gz topic -e -t /stats

# Expected output:
# real_time_factor: 0.95-1.05 (good)
# real_time_factor: <0.9 (simulation too slow)
# real_time_factor: >1.1 (simulation running fast)
```

### Optimization Techniques

1. **Simplify Collision Meshes**:
   ```xml
   <collision>
     <geometry>
       <box><size>0.5 0.3 0.7</size></box>  <!-- Simple box instead of mesh -->
     </geometry>
   </collision>
   ```

2. **Disable Unnecessary Contacts**:
   ```xml
   <surface>
     <contact>
       <collide_without_contact>true</collide_without_contact>
     </contact>
   </surface>
   ```

3. **Adjust Solver Iterations**:
   ```xml
   <dart>
     <solver>
       <iterations>25</iterations>  <!-- Reduce from default 50 -->
     </solver>
   </dart>
   ```

## Multi-Contact Scenarios

### Grasping Simulation

```xml
<!-- Gripper finger with high friction -->
<collision name="finger_collision">
  <geometry>
    <box><size>0.02 0.03 0.08</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.5</mu>
        <mu2>1.5</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e7</kp>
        <kd>1000</kd>
        <max_vel>0.001</max_vel>
      </ode>
    </contact>
  </surface>
</collision>
```

### Bipedal Walking

```xml
<!-- Foot contact with realistic friction -->
<collision name="foot_sole">
  <geometry>
    <box><size>0.15 0.08 0.02</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.9</mu>
        <mu2>0.8</mu2>
        <slip1>0.02</slip1>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>5e6</kp>
        <kd>500</kd>
        <max_vel>0.01</max_vel>
        <min_depth>0.002</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Debugging Physics Issues

### Common Problems

| Symptom | Cause | Solution |
|---------|-------|----------|
| **Robot explodes** | Timestep too large | Reduce `max_step_size` to 0.001 |
| **Jittery contacts** | `kp` too high | Lower contact stiffness |
| **Sinking through floor** | `kp` too low | Increase contact stiffness |
| **Slow simulation** | Too many iterations | Reduce solver iterations |
| **Unrealistic bounce** | `kd` too low | Increase contact damping |

### Enabling Contact Visualization

```bash
# Launch Gazebo with contact visualization
gz sim -v 4 world.sdf

# In Gazebo GUI:
# View → Contacts (show contact points and forces)
```

### Logging Contact Forces

```xml
<plugin filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
</plugin>
```

```python
# Subscribe to contact topic
import gz.transport

node = gz.transport.Node()

def contact_callback(msg):
    for contact in msg.contact:
        print(f"Contact force: {contact.wrench[0].force}")

node.subscribe("/world/humanoid_world/model/humanoid/link/foot/contacts",
               contact_callback)
```

## Advanced: Custom Physics Plugins

```cpp
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

namespace custom_physics {

class CustomGravity : public gz::sim::System,
                      public gz::sim::ISystemPreUpdate {
  public:
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override {
      // Apply custom forces
      auto link = _ecm.EntityByComponents(
          gz::sim::components::Name("torso"));

      gz::math::Vector3d customForce(0, 0, 10);  // Upward force
      _ecm.SetComponentData<gz::sim::components::ExternalForce>(
          link, customForce);
    }
};

}  // namespace custom_physics

GZ_ADD_PLUGIN(
    custom_physics::CustomGravity,
    gz::sim::System,
    custom_physics::CustomGravity::ISystemPreUpdate)
```

## Integration with ROS 2

### Publishing Joint States

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <update_rate>100</update_rate>
</plugin>
```

```bash
# View joint states
ros2 topic echo /joint_states
```

### Applying Forces via ROS

```python
from ros_gz_interfaces.srv import ApplyLinkWrench
from geometry_msgs.msg import Wrench

client = self.create_client(ApplyLinkWrench, '/world/my_world/apply_link_wrench')

request = ApplyLinkWrench.Request()
request.entity_name = 'humanoid::torso'
request.wrench.force.z = 100.0  # Apply 100N upward

client.call_async(request)
```

## Next Steps

- **[Chapter 3: Unity for HRI](./03-unity-hri)**: High-fidelity rendering for human interaction
- **[Chapter 4: Simulated Sensors](./04-simulated-sensors)**: Camera, lidar, IMU modeling
- **[Module 3: Isaac Sim](../03-isaac-robot-brain/01-isaac-sim-setup)**: GPU-accelerated physics

## Additional Resources

- [DART Physics Engine](https://dartsim.github.io/)
- [Gazebo Physics Tutorial](https://gazebosim.org/docs/latest/physics)
- [SDF Physics Reference](http://sdformat.org/spec?ver=1.9&elem=physics)
- [Contact Dynamics Theory](https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf)
