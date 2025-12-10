# Autonomous Navigation: Complete Pipeline

## Overview

This chapter integrates Isaac Sim, Isaac ROS Visual SLAM, and Nav2 into a complete autonomous navigation system for humanoid robots. We combine GPU-accelerated perception, real-time localization, and intelligent path planning to enable fully autonomous movement in complex environments.

**Complete System Stack**:
```
Voice Command → VLA Pipeline → Nav2 Actions → Isaac ROS VSLAM → Isaac Sim/Robot
```

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  VLA Voice Interface                    │
│  "Walk to the kitchen" → LLM Cognitive Planning        │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│              Action Service (Nav2 Backend)              │
│  NavigateToPose(x=5.0, y=3.0, theta=0.0)               │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│                    Nav2 Stack                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │   Global    │→ │    Local    │→ │ Controller  │     │
│  │   Planner   │  │   Planner   │  │   Server    │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│              Isaac ROS Visual SLAM                      │
│  Camera + IMU → 6-DOF Pose Estimation                  │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│          Robot (Isaac Sim or Physical)                  │
│  Execute velocity commands, update sensors              │
└─────────────────────────────────────────────────────────┘
```

## Complete Setup

### 1. Create Unified Workspace

```bash
# Create workspace
mkdir -p ~/autonomous_humanoid_ws/src
cd ~/autonomous_humanoid_ws/src

# Clone required packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/ros-planning/navigation2.git -b humble

# Install dependencies
cd ~/autonomous_humanoid_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### 2. Create Robot Package

```bash
cd ~/autonomous_humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_navigation \
  --dependencies rclpy nav2_msgs geometry_msgs isaac_ros_visual_slam
```

### 3. Robot Description (URDF)

Create `humanoid_navigation/urdf/humanoid.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.8"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.8"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.15 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="torso"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- IMU -->
  <link name="imu_link"/>

  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>
```

## Unified Configuration

### Navigation Parameters

Create `config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.2
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: -2.5
      decel_lim_theta: -3.2

planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "PointCloud2"

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "PointCloud2"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### EKF Sensor Fusion

Create `config/ekf_params.yaml`:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_time_offset: 0.0
    transform_timeout: 0.0
    print_diagnostics: true
    debug: false

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Visual SLAM odometry
    odom0: /visual_slam/odometry
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false

    # IMU data
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
    imu0_differential: false
    imu0_relative: false
```

## Unified Launch File

Create `launch/autonomous_navigation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('humanoid_navigation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml']),
            description='Full path to Nav2 parameters file'
        ),

        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open('/path/to/humanoid.urdf').read()
            }]
        ),

        # 2. Isaac ROS Visual SLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_imu_fusion': True,
                'enable_loop_closure': True,
                'publish_map_to_odom_tf': False,  # EKF handles this
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('camera/image_raw', '/camera/color/image_raw'),
                ('camera/camera_info', '/camera/color/camera_info'),
                ('imu/data', '/imu/data'),
                ('visual_slam/odometry', '/visual_slam/odometry'),
            ]
        ),

        # 3. Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'ekf_params.yaml']),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),

        # 4. Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items()
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'navigation.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
```

## VLA Integration

### Nav2 Action Service Implementation

Create `src/vla_core/services/nav2_action_service.py`:

```python
from src.vla_core.contracts.action_service import IActionService
from src.vla_core.models.robot_action import RobotAction, ActionType
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import asyncio

class Nav2ActionService(IActionService):
    """Production ActionService using Nav2 backend."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = Node('nav2_action_service')
        self.nav_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Spin node in background thread
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    async def execute_action(
        self,
        action: RobotAction,
        feedback_callback=None
    ) -> tuple[bool, str]:
        """Execute action using Nav2."""

        if action.action_type == ActionType.NAVIGATE:
            return await self._execute_navigate_nav2(action, feedback_callback)
        else:
            # Fall back to Pure Python for non-navigation actions
            return await super().execute_action(action, feedback_callback)

    async def _execute_navigate_nav2(
        self,
        action: RobotAction,
        feedback_callback
    ) -> tuple[bool, str]:
        """Execute navigation using Nav2 NavigateToPose action."""

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return (False, "Nav2 action server not available")

        # Create goal
        goal_msg = NavigateToPose.Goal()
        target = action.goal_message.get("target_pose", {})

        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target.get("x", 0.0))
        goal_msg.pose.pose.position.y = float(target.get("y", 0.0))
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        theta = target.get("theta", 0.0)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._handle_nav2_feedback(fb, feedback_callback)
        )

        # Wait for goal acceptance
        goal_handle = await self._wait_for_future(send_goal_future)

        if not goal_handle.accepted:
            return (False, "Navigation goal rejected by Nav2")

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await self._wait_for_future(result_future)

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return (True, "")
        else:
            return (False, f"Navigation failed with status: {result.status}")

    def _handle_nav2_feedback(self, nav2_feedback, vla_callback):
        """Convert Nav2 feedback to VLA feedback format."""
        if vla_callback:
            # Extract Nav2 feedback data
            fb = nav2_feedback.feedback

            # Convert to VLA format
            vla_feedback = {
                "progress_percent": self._estimate_progress(fb),
                "current_phase": "navigating",
                "distance_remaining": fb.distance_remaining,
                "estimated_time_remaining": fb.estimated_time_remaining.sec,
            }

            asyncio.create_task(vla_callback(vla_feedback))

    def _estimate_progress(self, nav2_feedback) -> float:
        """Estimate progress percentage from Nav2 feedback."""
        # Simple estimation based on distance
        initial_distance = getattr(self, '_initial_distance', 10.0)
        remaining = nav2_feedback.distance_remaining

        if initial_distance == 0:
            return 100.0

        progress = max(0.0, min(100.0, (1.0 - remaining / initial_distance) * 100.0))
        return progress

    async def _wait_for_future(self, future):
        """Convert ROS2 future to async/await."""
        while not future.done():
            await asyncio.sleep(0.01)
        return future.result()

    async def cancel_action(self, action_id: str) -> bool:
        """Cancel in-progress navigation."""
        # Implementation for goal cancellation
        pass

    def __del__(self):
        """Cleanup ROS2 node."""
        if hasattr(self, 'node'):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

### VLA Pipeline with Nav2

```python
# main.py
from src.vla_core.pipeline import VlaPipeline
from src.vla_core.services.nav2_action_service import Nav2ActionService
from src.vla_core.models.voice_command import VoiceCommand

async def main():
    # Use Nav2 backend for real robot
    action_service = Nav2ActionService()

    # Create pipeline
    pipeline = VlaPipeline(action_service=action_service)

    # Process voice command
    command = VoiceCommand(
        transcribed_text="Go to the kitchen",
        confidence=0.95
    )

    # Execute
    log = await pipeline.process_voice_command(command)

    print(f"Status: {log.final_status}")
    print(f"Actions executed: {len(log.action_plan.steps)}")

if __name__ == "__main__":
    asyncio.run(main())
```

## Testing the Complete System

### 1. Launch in Isaac Sim

```bash
# Terminal 1: Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh

# Terminal 2: Launch navigation stack
ros2 launch humanoid_navigation autonomous_navigation.launch.py use_sim_time:=true

# Terminal 3: Run VLA pipeline
python3 main.py
```

### 2. Test Navigation

```bash
# Send navigation goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}}"

# Or via VLA voice command
# Say: "Walk to coordinates x=2, y=1"
```

### 3. Monitor Performance

```bash
# Check topic rates
ros2 topic hz /visual_slam/odometry
ros2 topic hz /cmd_vel
ros2 topic hz /camera/color/image_raw

# Monitor transforms
ros2 run tf2_tools view_frames

# View logs
ros2 topic echo /rosout
```

## Performance Metrics

### Target Performance

| Metric | Target | Measured |
|--------|--------|----------|
| **Localization Rate** | 30 Hz | ___ Hz |
| **Controller Rate** | 20 Hz | ___ Hz |
| **Planning Time** | &lt;500ms | ___ ms |
| **Pose Accuracy** | &lt;5cm | ___ cm |
| **Success Rate** | >95% | ___% |

### Benchmarking Script

```python
# benchmark.py
import time
import rclpy
from geometry_msgs.msg import PoseStamped

class NavigationBenchmark:
    def __init__(self):
        self.start_time = None
        self.success_count = 0
        self.total_count = 0

    def run_benchmark(self, waypoints):
        for waypoint in waypoints:
            self.total_count += 1
            success = self.navigate_to(waypoint)
            if success:
                self.success_count += 1

        print(f"Success rate: {self.success_count/self.total_count*100:.1f}%")

    def navigate_to(self, waypoint):
        # Send goal and wait for result
        # Return True if succeeded
        pass
```

## Production Deployment

### Hardware Requirements

| Component | Specification |
|-----------|---------------|
| **Compute** | NVIDIA Jetson Orin or x86 + RTX GPU |
| **Camera** | RealSense D435i or ZED 2 (stereo) |
| **IMU** | VectorNav VN-100 or BNO085 |
| **LiDAR** | Optional: Velodyne VLP-16 |

### Deployment Checklist

- [ ] Calibrate all sensors (camera intrinsics, IMU-camera extrinsics)
- [ ] Test in controlled environment first
- [ ] Verify safety stops and emergency behaviors
- [ ] Tune costmap parameters for specific environment
- [ ] Set up monitoring and logging
- [ ] Create recovery procedures for common failures
- [ ] Implement battery monitoring and low-power returns

## Best Practices

1. **Always Test in Simulation First**: Use Isaac Sim to validate before hardware deployment
2. **Monitor GPU Utilization**: Ensure cuVSLAM runs efficiently
3. **Tune for Environment**: Different spaces require different costmap parameters
4. **Enable All Recovery Behaviors**: Robot should handle getting stuck
5. **Log Everything**: Capture rosbags for post-analysis
6. **Gradual Deployment**: Start with teleop, add waypoint navigation, then full autonomy

## Summary

You now have a complete autonomous navigation system that:
- ✅ Uses GPU-accelerated visual SLAM for localization
- ✅ Integrates Nav2 for intelligent path planning
- ✅ Connects to VLA pipeline for voice control
- ✅ Works in both simulation (Isaac Sim) and real hardware
- ✅ Provides comprehensive monitoring and debugging

This represents the state-of-the-art in humanoid robot navigation, combining the best of NVIDIA's Isaac ecosystem with ROS 2's proven navigation stack.

## Additional Resources

- [Isaac Sim Integration Examples](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- [Nav2 Best Practices](https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html)
- [cuVSLAM Performance Tuning](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html)
- [ROS 2 Production Deployment](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)
