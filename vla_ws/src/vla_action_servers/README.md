# VLA Action Servers

ROS 2 Action Servers for Vision-Language-Action (VLA) robot control.

## Overview

This package provides action servers that enable high-level robot control through ROS 2 actions. The servers simulate robot behaviors and can be replaced with real robot controllers.

## Action Servers

### 1. PickObject Server
- **Action**: `/pick_object`
- **Type**: `vla_interfaces/PickObject`
- **Description**: Simulates picking up objects with configurable grasp types and approach offsets

### 2. PlaceObject Server
- **Action**: `/place_object`
- **Type**: `vla_interfaces/PlaceObject`
- **Description**: Simulates placing objects at target poses

### 3. NavigateToPoint Server
- **Action**: `/navigate_to_point`
- **Type**: `vla_interfaces/NavigateToPoint`
- **Description**: Simulates base navigation to target poses with obstacle avoidance

### 4. InspectObject Server
- **Action**: `/inspect_object`
- **Type**: `vla_interfaces/InspectObject`
- **Description**: Simulates inspecting objects or regions with camera observations

## Building

```bash
cd vla_ws
colcon build --packages-select vla_action_servers
source install/setup.bash
```

## Running

### Launch All Servers

```bash
ros2 launch vla_action_servers vla_action_servers.launch.py
```

### Launch Individual Servers

```bash
# Pick server
ros2 run vla_action_servers pick_object_server

# Place server
ros2 run vla_action_servers place_object_server

# Navigate server
ros2 run vla_action_servers navigate_to_point_server

# Inspect server
ros2 run vla_action_servers inspect_object_server
```

## Testing Action Servers

### Test with ros2 action command

```bash
# Send a pick goal
ros2 action send_goal /pick_object vla_interfaces/action/PickObject "{object_id: 'red_cube', approach_offset: {x: 0.0, y: 0.0, z: 0.1}, grasp_type: 'top', max_force: 50.0}" --feedback

# Send a navigate goal
ros2 action send_goal /navigate_to_point vla_interfaces/action/NavigateToPoint "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, tolerance_xy: 0.1, tolerance_theta: 0.15, max_speed: 0.5, avoid_obstacles: true}" --feedback
```

### Test with Python Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vla_interfaces.action import PickObject

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self._action_client = ActionClient(self, PickObject, '/pick_object')

    def send_goal(self):
        goal_msg = PickObject.Goal()
        goal_msg.object_id = 'red_cube'
        goal_msg.grasp_type = 'top'
        goal_msg.max_force = 50.0

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    client = TestClient()
    future = client.send_goal()
    rclpy.spin_until_future_complete(client, future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with VLA Pipeline

The VLA pipeline (in `src/vla_core`) connects to these action servers through the `RosInterface` class:

```python
from vla_core.execution.ros_interface import RosInterface

# Create interface (use_mock=False for real servers)
ros_interface = RosInterface(use_mock=False)
await ros_interface.connect()

# Execute action
from vla_core.models import RobotAction, ActionType
action = RobotAction(
    action_type=ActionType.PICK,
    ros_action_name='/pick_object',
    goal_message={'object_id': 'red_cube', 'grasp_type': 'top'},
    timeout=30.0
)

success, error = await ros_interface.execute_action(action)
```

## Server Implementation Details

Each server:
- Accepts goals with validation
- Provides real-time feedback with progress updates
- Supports cancellation mid-execution
- Returns success/failure results
- Runs in a multi-threaded executor for concurrent requests

## Future Enhancements

- Replace simulated execution with real robot controllers
- Add collision checking and safety validation
- Integrate with MoveIt for motion planning
- Add visual servoing for manipulation tasks
- Implement recovery behaviors

## Dependencies

- rclpy
- vla_interfaces
- geometry_msgs
- std_msgs

## License

MIT