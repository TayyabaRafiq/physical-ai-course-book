# Nav2 Integration for Autonomous Navigation

## Overview

Nav2 (Navigation 2) is the ROS 2 navigation framework that enables robots to autonomously navigate from point A to point B while avoiding obstacles. It integrates with Isaac ROS Visual SLAM for localization and provides a complete navigation stack for humanoid robots.

**Core Capabilities**:
- Global and local path planning
- Dynamic obstacle avoidance
- Recovery behaviors
- Behavior trees for complex navigation logic
- Multi-robot support

## Nav2 Architecture

```
┌─────────────────────────────────────────────────┐
│              Nav2 Stack                         │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐  │
│  │ Global   │───▶│  Local   │───▶│Controller│  │
│  │ Planner  │    │ Planner  │    │  Server  │  │
│  └──────────┘    └──────────┘    └──────────┘  │
│       │                │               │        │
│       ▼                ▼               ▼        │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐  │
│  │  Costmap │    │  Costmap │    │ Recovery │  │
│  │  (Global)│    │  (Local) │    │ Behaviors│  │
│  └──────────┘    └──────────┘    └──────────┘  │
│                                                 │
├─────────────────────────────────────────────────┤
│  Localization: Isaac ROS VSLAM / AMCL          │
├─────────────────────────────────────────────────┤
│  Sensors: Camera, Lidar, Depth, IMU            │
└─────────────────────────────────────────────────┘
```

## Installation

### Install Nav2

```bash
# Install Nav2 packages
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-nav2-msgs \
                 ros-humble-turtlebot3-gazebo

# Install dependencies
sudo apt install ros-humble-robot-localization \
                 ros-humble-slam-toolbox
```

### Verify Installation

```bash
# Check Nav2 packages
ros2 pkg list | grep nav2

# Expected output includes:
# nav2_bringup
# nav2_controller
# nav2_planner
# nav2_behaviors
# nav2_bt_navigator
```

## Configuration

### Nav2 Parameters

Create `nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5  # Humanoid walking speed
      min_vel_y: -0.2
      max_vel_y: 0.2
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 2.5
      acc_lim_y: 2.5
      acc_lim_theta: 3.2

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false  # Use Dijkstra
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

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      width: 50
      height: 50
      resolution: 0.05
      robot_radius: 0.3  # Humanoid footprint
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan depth_camera
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      width: 10
      height: 10
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan depth_camera
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### Robot Footprint

For humanoid robots, define a circular footprint:

```yaml
# footprint.yaml
robot_radius: 0.3  # 30cm radius for humanoid base
footprint_padding: 0.05

# Or polygon footprint for more accuracy
footprint: [
  [0.25, 0.15],   # front-right
  [0.25, -0.15],  # front-left
  [-0.25, -0.15], # back-left
  [-0.25, 0.15]   # back-right
]
```

## Launch Files

### Basic Nav2 Launch

```python
# launch/nav2_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # Launch Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file': '/path/to/nav2_params.yaml',
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()
        ),
    ])
```

### Nav2 with Isaac VSLAM

```python
# launch/nav2_vslam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS Visual SLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_imu_fusion': True,
                'publish_map_to_odom_tf': True,
            }],
            remappings=[
                ('visual_slam/odometry', '/odometry/vslam'),
            ]
        ),

        # Robot Localization (EKF fusion)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['/path/to/ekf_params.yaml'],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),

        # Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/path/to/nav2_bringup.launch.py'
            )
        ),
    ])
```

## Running Nav2

### 1. Build Map (Optional)

For static environments, build a map first:

```bash
# Launch SLAM Toolbox for mapping
ros2 launch slam_toolbox online_async_launch.py

# Drive robot around to build map
# Save map when done
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 2. Launch Navigation Stack

```bash
# With pre-built map
ros2 launch nav2_bringup bringup_launch.py \
  params_file:=/path/to/nav2_params.yaml \
  map:=/path/to/my_map.yaml

# Without map (using VSLAM)
ros2 launch my_robot_nav nav2_vslam.launch.py
```

### 3. Set Initial Pose

```bash
# Via command line
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}"

# Or use RViz "2D Pose Estimate" button
```

### 4. Send Navigation Goal

```bash
# Via command line
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"

# Or use RViz "Nav2 Goal" button
```

## RViz Visualization

### Configure RViz

Add these displays to RViz:

```yaml
Displays:
  - Class: rviz_default_plugins/Map
    Topic: /map
  - Class: rviz_default_plugins/Path
    Topic: /plan  # Global path
  - Class: rviz_default_plugins/Path
    Topic: /local_plan  # Local path
  - Class: nav2_rviz_plugins/GoalPose
    Topic: /goal_pose
  - Class: rviz_default_plugins/Map
    Topic: /global_costmap/costmap
  - Class: rviz_default_plugins/Map
    Topic: /local_costmap/costmap
  - Class: rviz_default_plugins/TF
```

### Launch RViz with Nav2

```bash
ros2 launch nav2_bringup rviz_launch.py
```

## Action API

### NavigateToPose Action

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}m")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation completed')

# Usage
rclpy.init()
navigator = Nav2Client()
navigator.send_goal(2.0, 1.0, 0.0)
rclpy.spin(navigator)
```

## Integration with VLA Pipeline

### VLA ActionService with Nav2

```python
from src.vla_core.contracts.action_service import IActionService
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2ActionService(IActionService):
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('nav2_action_service')
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    async def execute_action(self, action, feedback_callback):
        if action.action_type == ActionType.NAVIGATE:
            return await self._navigate_with_nav2(action, feedback_callback)
        # ... other action types

    async def _navigate_with_nav2(self, action, feedback_callback):
        goal = NavigateToPose.Goal()
        target = action.goal_message.get("target_pose", {})

        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = target.get("x", 0.0)
        goal.pose.pose.position.y = target.get("y", 0.0)

        # Send goal and await result
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: feedback_callback({
                "progress_percent": (1.0 - fb.distance_remaining / initial_distance) * 100,
                "distance_remaining": fb.distance_remaining
            })
        )

        result = await future
        return (result.status == GoalStatus.SUCCEEDED, "")
```

## Behavior Trees

Nav2 uses Behavior Trees (BT) for complex navigation logic:

```xml
<!-- bt_navigator.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePath">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <SequenceStar name="RecoveryActions">
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
        </SequenceStar>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Performance Tuning

### Costmap Resolution

```yaml
# High resolution (slower, more accurate)
resolution: 0.025  # 2.5cm

# Low resolution (faster, less accurate)
resolution: 0.1    # 10cm
```

### Controller Frequency

```yaml
# High frequency (smoother, more CPU)
controller_frequency: 20.0

# Low frequency (less CPU)
controller_frequency: 10.0
```

### Planner Tolerance

```yaml
# Strict (precise goal reaching)
tolerance: 0.1

# Relaxed (faster planning)
tolerance: 0.5
```

## Common Issues

### Issue: Robot not moving

**Solutions**:
1. Check velocity limits in controller params
2. Verify costmap is populated
3. Ensure localization is active
   ```bash
   ros2 topic echo /tf
   ```

### Issue: Robot oscillating

**Solutions**:
1. Reduce controller frequency
2. Increase path tolerance
3. Tune DWB parameters (vx_samples, vy_samples)

### Issue: Planning failures

**Solutions**:
1. Increase planner timeout
2. Check for unknown space in costmap
3. Verify obstacle layers are configured

## Best Practices

1. **Tune for Robot Dynamics**: Match velocity limits to humanoid walking capabilities
2. **Use Multiple Sensors**: Combine lidar, depth camera, and visual odometry
3. **Test Recovery Behaviors**: Ensure robot can recover from getting stuck
4. **Monitor Performance**: Use `ros2 topic hz` to verify update rates
5. **Visualize Costmaps**: Always check costmaps in RViz during tuning

## Next Steps

- **[Chapter 4: Autonomous Navigation](./04-autonomous-navigation)**: Complete end-to-end navigation system

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [Behavior Trees in Nav2](https://navigation.ros.org/behavior_trees/index.html)
- [Nav2 Plugins](https://navigation.ros.org/plugins/index.html)
