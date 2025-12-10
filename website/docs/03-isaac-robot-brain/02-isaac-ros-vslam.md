# Isaac ROS Visual SLAM

## Overview

Isaac ROS Visual SLAM (cuVSLAM) provides GPU-accelerated visual-inertial odometry and mapping for robot localization and navigation. It fuses camera images with IMU data to estimate the robot's pose and build 3D maps in real-time.

**Key Capabilities**:
- Real-time 6-DOF pose estimation
- 3D landmark mapping
- Loop closure detection
- GPU acceleration via CUDA
- ROS 2 native integration

## Why Visual SLAM?

### Advantages over Wheel Odometry

| Feature | Wheel Odometry | Visual SLAM |
|---------|----------------|-------------|
| **Accuracy** | Degrades over distance | Bounded error via loop closure |
| **Terrain** | Fails on slippery surfaces | Works anywhere with features |
| **Drift** | Unbounded accumulation | Corrected by visual landmarks |
| **Cost** | Low (encoders) | Medium (camera + IMU) |

### Use Cases for Humanoid Robots

- **Bipedal Navigation**: Wheel odometry not applicable
- **Dynamic Environments**: Adapt to changing layouts
- **Multi-Floor Buildings**: 3D mapping for stairs
- **GPS-Denied**: Indoor localization

## Architecture

```
┌─────────────┐    ┌─────────────┐
│   Camera    │───▶│             │
│ (Stereo/RGB)│    │  cuVSLAM    │──▶ Pose Estimate
└─────────────┘    │   Engine    │    (x, y, z, quat)
                   │             │
┌─────────────┐    │ (GPU-Accel) │──▶ 3D Landmarks
│     IMU     │───▶│             │    (Point Cloud)
└─────────────┘    └─────────────┘
                         │
                         ▼
                   ┌─────────────┐
                   │ Loop Closure│
                   │  Detection  │
                   └─────────────┘
```

## Installation

### Prerequisites

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# CUDA 11.8+ (verify)
nvcc --version

# Isaac ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
```

### Install Isaac ROS Visual SLAM

```bash
# Clone repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Install dependencies
cd ~/isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --packages-up-to isaac_ros_visual_slam

# Source workspace
source install/setup.bash
```

### Verify Installation

```bash
# Check package
ros2 pkg list | grep isaac_ros_visual_slam

# Expected output:
# isaac_ros_visual_slam
```

## Configuration

### Camera Setup

Isaac ROS VSLAM supports multiple camera types:

#### 1. Stereo Camera (Recommended)

```yaml
# stereo_camera.yaml
camera:
  type: stereo
  left_topic: /camera/left/image_raw
  right_topic: /camera/right/image_raw
  left_info_topic: /camera/left/camera_info
  right_info_topic: /camera/right/camera_info
  baseline: 0.12  # meters (distance between cameras)
```

#### 2. Monocular Camera + IMU

```yaml
# mono_imu.yaml
camera:
  type: monocular
  image_topic: /camera/image_raw
  info_topic: /camera/camera_info

imu:
  topic: /imu/data
  required: true
```

#### 3. RGB-D Camera

```yaml
# rgbd.yaml
camera:
  type: rgbd
  rgb_topic: /camera/color/image_raw
  depth_topic: /camera/depth/image_raw
  info_topic: /camera/color/camera_info
```

### Visual SLAM Parameters

```yaml
# vslam_params.yaml
visual_slam_node:
  ros__parameters:
    # Input topics
    camera_info_topic: /camera/camera_info
    image_topic: /camera/image_raw
    imu_topic: /imu/data

    # Output topics
    pose_topic: /visual_slam/pose
    odometry_topic: /visual_slam/odometry
    map_topic: /visual_slam/map

    # Algorithm parameters
    enable_imu_fusion: true
    enable_loop_closure: true
    enable_observations_view: true
    enable_landmarks_view: true

    # Performance tuning
    num_cameras: 1
    max_features: 5000
    min_num_images: 10

    # Map management
    publish_map_to_odom_tf: true
    publish_odom_to_base_tf: true
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
```

## Launch Files

### Basic VSLAM Launch

```python
# launch/vslam_basic.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam_node',
            parameters=[{
                'enable_imu_fusion': True,
                'enable_loop_closure': True,
                'num_cameras': 1,
            }],
            remappings=[
                ('camera/image_raw', '/camera/color/image_raw'),
                ('camera/camera_info', '/camera/color/camera_info'),
                ('imu/data', '/imu/data'),
            ]
        )
    ])
```

### VSLAM with Visualization

```python
# launch/vslam_rviz.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('isaac_ros_visual_slam'),
        'rviz', 'vslam.rviz'
    )

    return LaunchDescription([
        # Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam_node',
            parameters=['/path/to/vslam_params.yaml'],
        ),

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
```

## Running Visual SLAM

### With Physical Camera

```bash
# Terminal 1: Launch camera driver
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: Launch Visual SLAM
ros2 launch isaac_ros_visual_slam vslam_basic.launch.py

# Terminal 3: Visualize in RViz
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam)/share/isaac_ros_visual_slam/rviz/vslam.rviz
```

### With Rosbag Playback

```bash
# Terminal 1: Launch VSLAM
ros2 launch isaac_ros_visual_slam vslam_basic.launch.py

# Terminal 2: Play recorded data
ros2 bag play my_robot_data.db3
```

### With Isaac Sim

```python
# isaac_sim_vslam.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS2 bridge
enable_extension("omni.isaac.ros2_bridge")

# Create world and robot
world = World()
# ... add robot with camera and IMU

# VSLAM will receive data via ROS2 bridge
world.reset()
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## Output Topics

### Pose Estimate

```bash
# View pose
ros2 topic echo /visual_slam/pose
```

**Message Type**: `geometry_msgs/msg/PoseStamped`

```yaml
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "map"
pose:
  position: {x: 1.23, y: 0.45, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
```

### Odometry

```bash
# View odometry
ros2 topic echo /visual_slam/odometry
```

**Message Type**: `nav_msgs/msg/Odometry`

Includes pose, twist (velocity), and covariance estimates.

### 3D Map (Point Cloud)

```bash
# View landmarks
ros2 topic echo /visual_slam/landmarks
```

**Message Type**: `sensor_msgs/msg/PointCloud2`

Contains 3D positions of mapped features.

## TF Tree

Visual SLAM publishes transforms:

```
map → odom → base_link → camera_link
                      → imu_link
```

View TF tree:

```bash
# Install tf2_tools
sudo apt install ros-humble-tf2-tools

# Generate TF tree
ros2 run tf2_tools view_frames

# Open frames.pdf
```

## Performance Tuning

### GPU Utilization

Monitor GPU usage:

```bash
# Watch GPU utilization
watch -n 0.5 nvidia-smi
```

Target: 60-80% GPU utilization for optimal performance.

### Feature Detection

Adjust feature count based on environment:

```yaml
# Rich features (indoor)
max_features: 5000

# Sparse features (outdoor)
max_features: 2000
```

### Frame Rate

Balance between accuracy and latency:

```yaml
# High accuracy (10-20 FPS)
process_every_n_frames: 1

# Lower latency (30+ FPS possible)
process_every_n_frames: 2
```

## Troubleshooting

### Issue: Poor tracking

**Symptoms**: Pose jumps or tracking lost

**Solutions**:
1. Check camera calibration
   ```bash
   ros2 topic echo /camera/camera_info
   ```
2. Ensure sufficient lighting
3. Increase `max_features`
4. Verify IMU data
   ```bash
   ros2 topic hz /imu/data  # Should be >100 Hz
   ```

### Issue: High drift

**Symptoms**: Pose drifts over time

**Solutions**:
1. Enable loop closure
   ```yaml
   enable_loop_closure: true
   ```
2. Use stereo camera instead of mono
3. Improve camera calibration
4. Add more visual features to environment

### Issue: GPU out of memory

**Symptoms**: CUDA OOM errors

**Solutions**:
1. Reduce `max_features`
2. Lower image resolution
3. Disable `enable_observations_view`

## Integration with Nav2

Visual SLAM provides localization for Nav2 navigation:

```python
# launch/vslam_nav2.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Visual SLAM for localization
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            parameters=[{'publish_map_to_odom_tf': True}],
        ),

        # Nav2 for path planning
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            parameters=[{
                'use_sim_time': False,
                'localization': 'vslam'  # Use VSLAM instead of AMCL
            }]
        ),
    ])
```

## Saving and Loading Maps

### Save Map

```bash
# Trigger map save service
ros2 service call /visual_slam/save_map \
  isaac_ros_visual_slam/srv/SaveMap \
  "{map_url: '/path/to/my_map.bin'}"
```

### Load Map

```bash
# Launch with pre-existing map
ros2 launch isaac_ros_visual_slam vslam_basic.launch.py \
  map_file:=/path/to/my_map.bin
```

## Best Practices

1. **Camera Calibration**: Always calibrate cameras using `camera_calibration` package
2. **IMU-Camera Sync**: Ensure timestamp synchronization between sensors
3. **Feature-Rich Environment**: Add visual markers if environment is textureless
4. **Loop Closure**: Enable for long-duration operations
5. **Map Management**: Save maps for known environments to speed up localization

## Comparison with Other SLAM Methods

| Method | Accuracy | Speed | GPU Required | Environment |
|--------|----------|-------|--------------|-------------|
| **cuVSLAM** | High | Very Fast | Yes | Feature-rich |
| **ORB-SLAM3** | High | Medium | No | Any |
| **RTAB-Map** | Medium | Medium | No | Any |
| **Cartographer** | Medium | Fast | No | Lidar needed |

cuVSLAM excels in GPU-accelerated environments with good visual features.

## Next Steps

- **[Chapter 3: Nav2 Integration](./03-nav2-integration)**: Connect VSLAM to navigation stack
- **[Chapter 4: Autonomous Navigation](./04-autonomous-navigation)**: Complete autonomous system

## Additional Resources

- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [cuVSLAM Technical Paper](https://developer.nvidia.com/blog/accelerating-visual-slam-with-nvidia-cuvslam/)
- [Camera Calibration Guide](http://wiki.ros.org/camera_calibration)
- [IMU Calibration](https://github.com/ethz-asl/kalibr)
