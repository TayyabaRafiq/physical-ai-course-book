# Simulated Sensors

## Overview

Simulated sensors bridge the gap between virtual environments and real-world robotics by generating realistic sensor data (images, point clouds, IMU readings) from digital twins. Accurate sensor simulation enables algorithm development and testing without physical hardware.

## Why Simulate Sensors?

| Benefit | Description |
|---------|-------------|
| **Safety** | Test perception algorithms without robot |
| **Ground Truth** | Perfect labels for ML training |
| **Edge Cases** | Simulate rare scenarios (fog, darkness, sensor failures) |
| **Repeatability** | Identical conditions across experiments |
| **Cost** | No sensor hardware required |

## Sensor Categories

### 1. Exteroceptive Sensors

Measure the environment:
- **Camera** (RGB, depth, thermal)
- **Lidar** (2D/3D laser scanning)
- **Radar** (range and velocity)
- **Ultrasonic** (proximity)

### 2. Proprioceptive Sensors

Measure robot state:
- **IMU** (accelerometer, gyroscope)
- **Encoders** (joint positions)
- **Force/Torque** (contact sensing)

## Camera Simulation

### RGB Camera (Gazebo)

```xml
<sensor name="camera" type="camera">
  <pose>0.1 0 0.5 0 0 0</pose>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.396</horizontal_fov>  <!-- 80 degrees -->
    <image>
      <width>1280</width>
      <height>720</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- Sensor noise -->
    </noise>
  </camera>
  <plugin filename="gz-sim-camera-system"
          name="gz::sim::systems::Camera">
  </plugin>
</sensor>
```

**Published Topic**:
```bash
ros2 topic echo /camera/image_raw
# Type: sensor_msgs/Image
```

### Depth Camera

```xml
<sensor name="depth_camera" type="depth_camera">
  <update_rate>20</update_rate>
  <camera>
    <horizontal_fov>1.396</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin filename="gz-sim-depth-camera-system"
          name="gz::sim::systems::DepthCamera">
    <ros>
      <remapping>depth_camera/image_raw:=/depth/image</remapping>
      <remapping>depth_camera/points:=/depth/points</remapping>
    </ros>
  </plugin>
</sensor>
```

**Output**:
- `/depth/image`: Depth image (32FC1)
- `/depth/points`: PointCloud2

### RGBD Camera (Kinect-style)

```xml
<sensor name="rgbd_camera" type="rgbd_camera">
  <update_rate>15</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.5</near>
      <far>5.0</far>
    </clip>
  </camera>
  <plugin filename="gz-sim-rgbd-camera-system"
          name="gz::sim::systems::RgbdCamera">
  </plugin>
</sensor>
```

## Lidar Simulation

### 2D Lidar (Laser Scanner)

```xml
<sensor name="lidar_2d" type="gpu_lidar">
  <pose>0 0 0.3 0 0 0</pose>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
  <plugin filename="gz-sim-lidar-system"
          name="gz::sim::systems::Lidar">
    <ros>
      <remapping>~/scan:=/scan</remapping>
    </ros>
  </plugin>
</sensor>
```

**ROS Message**:
```bash
ros2 topic echo /scan
# Type: sensor_msgs/LaserScan
# Fields: ranges[], intensities[], angle_min, angle_max, angle_increment
```

### 3D Lidar (Velodyne-style)

```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>  <!-- 16-beam lidar -->
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.5</min>
      <max>100.0</max>
    </range>
  </lidar>
  <plugin filename="gz-sim-lidar-system"
          name="gz::sim::systems::Lidar">
    <ros>
      <remapping>~/points:=/velodyne/points</remapping>
    </ros>
  </plugin>
</sensor>
```

**Output**: `sensor_msgs/PointCloud2`

## IMU Simulation

```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.1 0 0 0</pose>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.009</stddev>  <!-- 0.5 deg/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.009</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.009</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>  <!-- 0.017 m/s² -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin filename="gz-sim-imu-system"
          name="gz::sim::systems::Imu">
    <ros>
      <remapping>~/imu:=/imu/data</remapping>
    </ros>
  </plugin>
</sensor>
```

**ROS Message**:
```python
# sensor_msgs/Imu
orientation: [x, y, z, w]  # Quaternion
angular_velocity: [x, y, z]  # rad/s
linear_acceleration: [x, y, z]  # m/s²
```

## Force/Torque Sensor

```xml
<sensor name="force_torque" type="force_torque">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
  <plugin filename="gz-sim-force-torque-system"
          name="gz::sim::systems::ForceTorque">
    <ros>
      <remapping>~/wrench:=/wrist/force_torque</remapping>
    </ros>
  </plugin>
</sensor>
```

**Use Case**: Gripper force feedback, collision detection

## Contact Sensor

```xml
<sensor name="contact_sensor" type="contact">
  <contact>
    <collision>finger_collision</collision>
  </contact>
  <update_rate>100</update_rate>
  <plugin filename="gz-sim-contact-system"
          name="gz::sim::systems::Contact">
  </plugin>
</sensor>
```

**Access Contacts**:
```python
def contact_callback(msg):
    if len(msg.contact) > 0:
        print(f"Contact detected with {msg.contact[0].collision2}")
        force = msg.contact[0].wrench[0].force
        print(f"Force: {force.z} N")
```

## GPS Sensor

```xml
<sensor name="gps" type="gps">
  <update_rate>10</update_rate>
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>2.0</stddev>  <!-- 2m accuracy -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>4.0</stddev>  <!-- 4m vertical -->
        </noise>
      </vertical>
    </position_sensing>
  </gps>
  <plugin filename="gz-sim-gps-system"
          name="gz::sim::systems::Gps">
  </plugin>
</sensor>
```

## Sensor Noise Models

### Gaussian Noise

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- Standard deviation -->
</noise>
```

### Salt-and-Pepper Noise (Camera)

```xml
<noise>
  <type>salt_pepper</type>
  <salt_pepper_ratio>0.01</salt_pepper_ratio>  <!-- 1% of pixels -->
</noise>
```

### Custom Noise (Python)

```python
import numpy as np

def add_realistic_noise(image, noise_level=0.02):
    """Add Gaussian + salt-and-pepper noise."""
    # Gaussian noise
    gaussian = np.random.normal(0, noise_level, image.shape)
    noisy = image + gaussian

    # Salt-and-pepper
    salt_pepper = np.random.rand(*image.shape)
    noisy[salt_pepper < 0.005] = 0  # Pepper
    noisy[salt_pepper > 0.995] = 1  # Salt

    return np.clip(noisy, 0, 1)
```

## Sensor Placement Best Practices

### Camera Placement

```xml
<!-- Head-mounted camera -->
<joint name="camera_joint" type="fixed">
  <parent>head</parent>
  <child>camera_link</child>
  <origin xyz="0.1 0 0.05" rpy="0 0.2 0"/>  <!-- Slight downward tilt -->
</joint>
```

**Guidelines**:
- **Height**: Eye-level for humanoids (~1.6m)
- **Tilt**: 10-15° downward for floor visibility
- **Stereo Baseline**: 6-12cm for depth perception

### Lidar Mounting

```xml
<!-- Torso-mounted 2D lidar -->
<joint name="lidar_joint" type="fixed">
  <parent>torso</parent>
  <child>lidar_link</child>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Waist height -->
</joint>
```

### IMU Placement

```xml
<!-- IMU at center of mass -->
<joint name="imu_joint" type="fixed">
  <parent>torso</parent>
  <child>imu_link</child>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- Near CoM -->
</joint>
```

## Processing Sensor Data

### Camera Image Processing (Python)

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process (e.g., edge detection)
        edges = cv2.Canny(cv_image, 100, 200)

        # Display
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)
```

### Point Cloud Processing

```python
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def pointcloud_callback(msg: PointCloud2):
    # Extract points
    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])

    # Filter by distance
    filtered = [p for p in points if np.linalg.norm(p) < 5.0]
    print(f"Found {len(filtered)} points within 5m")
```

## Unity Sensor Simulation

### Depth Camera (Unity)

```csharp
using UnityEngine;

public class DepthCamera : MonoBehaviour
{
    public Camera depthCamera;
    RenderTexture depthTexture;

    void Start()
    {
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
        depthTexture = new RenderTexture(640, 480, 24, RenderTextureFormat.RFloat);
        depthCamera.targetTexture = depthTexture;
        depthCamera.SetReplacementShader(Shader.Find("Custom/DepthShader"), "");
    }

    public float GetDepthAt(int x, int y)
    {
        RenderTexture.active = depthTexture;
        Texture2D tex = new Texture2D(1, 1, TextureFormat.RFloat, false);
        tex.ReadPixels(new Rect(x, y, 1, 1), 0, 0);
        tex.Apply();
        return tex.GetPixel(0, 0).r;
    }
}
```

### Synthetic Lidar (Unity)

```csharp
public class SyntheticLidar : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 30f;

    public float[] Scan()
    {
        float[] ranges = new float[numRays];
        float angleStep = 360f / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, maxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxRange;
            }
        }

        return ranges;
    }
}
```

## Sensor Calibration

### Camera Intrinsics

```yaml
# camera_info.yaml
image_width: 1280
image_height: 720
camera_matrix:
  rows: 3
  cols: 3
  data: [700.0, 0.0, 640.0,
         0.0, 700.0, 360.0,
         0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.2, 0.05, 0.0, 0.0, 0.0]  # k1, k2, p1, p2, k3
```

## Integration with VLA Pipeline

Sensors provide input to the **Perception Layer**:

```python
class PerceptionNode(Node):
    def __init__(self, action_service):
        super().__init__('perception_node')
        self.action_service = action_service

        # Subscribe to sensors
        self.create_subscription(Image, '/camera/image', self.on_image, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_image(self, msg):
        # Detect objects
        objects = self.detect_objects(msg)

        # Update action service with object locations
        self.action_service.update_object_database(objects)
```

## Next Steps

- **[Module 3: Isaac Sim](../03-isaac-robot-brain/01-isaac-sim-setup)**: GPU-accelerated sensor simulation
- **[Module 4: VLA Integration](../04-vla-integration/01-voice-interface)**: Use sensor data for action planning

## Additional Resources

- [Gazebo Sensors](https://gazebosim.org/docs/latest/sensors)
- [ROS 2 Sensor Messages](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [cv_bridge Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/CvBridge.html)
- [Isaac Sim Sensors](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/index.html)
