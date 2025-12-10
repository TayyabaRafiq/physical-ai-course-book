# ROS 2 Extension (Optional)

## Overview

The VLA Integration MVP uses Pure Python execution by default. This page explains how to **optionally** extend the system with ROS 2 for real robot control.

:::info MVP Complete Without ROS 2
The VLA system is fully functional using the Pure Python ActionService. ROS 2 integration is only needed for:
- Physical robot deployment
- Hardware-in-the-loop testing
- Multi-robot coordination
- Physics-accurate simulation (Gazebo/Isaac Sim)
:::

## Why Add ROS 2?

### When Pure Python is Sufficient

✅ **Use Pure Python** for:
- Voice command demonstration
- Algorithm development
- UI/UX testing
- Educational purposes
- Rapid prototyping
- Cross-platform deployment

### When ROS 2 is Needed

🔧 **Add ROS 2** for:
- Physical robot deployment
- Real-time motor control
- Sensor integration (LIDAR, cameras, IMU)
- Gazebo physics simulation
- Multi-robot systems
- Industry-standard robotics stack

## Architecture Comparison

### Pure Python (Current MVP)

```
VoiceInterface → IntentParser → ActionPlanner
                                     ↓
                              PlanValidator
                                     ↓
                           ExecutionMonitor
                                     ↓
                             ActionService (Pure Python)
                                     ↓
                              Simulated Progress
```

### With ROS 2 Extension

```
VoiceInterface → IntentParser → ActionPlanner
                                     ↓
                              PlanValidator
                                     ↓
                           ExecutionMonitor
                                     ↓
                        ActionService (ROS 2 Backend)
                                     ↓
                              ROS 2 Actions
                                     ↓
                         [Physical Robot / Gazebo]
```

## Implementation Strategy

### Step 1: Install ROS 2

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Build Action Interfaces

The VLA workspace already contains ROS 2 action definitions:

```bash
cd vla_ws
colcon build --packages-select vla_interfaces
source install/setup.bash
```

**Action Definitions**:
- `PickObject.action`
- `PlaceObject.action`
- `NavigateToPoint.action`
- `InspectObject.action`

### Step 3: Create RosActionService

```python
# src/vla_core/execution/ros_action_service.py

import rclpy
from rclpy.action import ActionClient
from vla_interfaces.action import PickObject, PlaceObject, NavigateToPoint

from ..contracts.interfaces import IActionService
from ..models.robot_action import RobotAction, ActionType

class RosActionService(IActionService):
    """ROS 2 backend for ActionService."""

    def __init__(self, node_name="vla_action_client"):
        rclpy.init()
        self.node = rclpy.create_node(node_name)

        # Create action clients
        self.pick_client = ActionClient(self.node, PickObject, '/pick_object')
        self.place_client = ActionClient(self.node, PlaceObject, '/place_object')
        self.navigate_client = ActionClient(self.node, NavigateToPoint, '/navigate_to_point')

    async def execute_action(self, action: RobotAction, feedback_callback=None):
        """Execute action via ROS 2 action server."""

        # Select appropriate client
        if action.action_type == ActionType.PICK:
            client = self.pick_client
            goal_msg = self._create_pick_goal(action)
        elif action.action_type == ActionType.PLACE:
            client = self.place_client
            goal_msg = self._create_place_goal(action)
        elif action.action_type == ActionType.NAVIGATE:
            client = self.navigate_client
            goal_msg = self._create_navigate_goal(action)
        else:
            return (False, f"Unsupported action type: {action.action_type}")

        # Wait for server
        if not client.wait_for_server(timeout_sec=5.0):
            return (False, f"Action server not available: {client.action_name}")

        # Send goal
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self._handle_ros_feedback(msg, feedback_callback)
        )

        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            return (False, "Goal rejected by action server")

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await result_future

        return (result.result.success, result.result.error_message)

    def _create_pick_goal(self, action: RobotAction):
        """Convert RobotAction to PickObject goal."""
        goal = PickObject.Goal()
        goal.object_id = action.goal_message.get("object_id", "")
        goal.grasp_type = action.goal_message.get("grasp_type", "top")

        # Optional: approach offset
        offset = action.goal_message.get("approach_offset", {})
        goal.approach_offset.z = offset.get("z", 0.1)

        return goal

    def _handle_ros_feedback(self, ros_feedback, callback):
        """Convert ROS feedback to ActionService format."""
        if callback:
            feedback = {
                "progress_percent": ros_feedback.progress_percent,
                "current_phase": ros_feedback.current_phase,
                "status_message": ros_feedback.status_message
            }
            callback(feedback)
```

### Step 4: Configure Pipeline

```python
# src/vla_core/pipeline/vla_pipeline.py

from ..execution import ActionService, RosActionService
from ..utils.config import get_config

class VlaPipeline:
    def __init__(self, use_ros=False):
        # ... other components ...

        # Select execution backend
        if use_ros:
            action_service = RosActionService()
        else:
            action_service = ActionService()  # Pure Python

        self.execution_monitor = ExecutionMonitor(action_service)
```

### Step 5: CLI Configuration

```bash
# .env
USE_ROS_BACKEND=false  # Default: Pure Python
ROS_DOMAIN_ID=0        # Only used if USE_ROS_BACKEND=true
```

```python
# src/cli/vla_cli.py

@click.option('--use-ros/--no-ros', default=False, help='Use ROS 2 backend')
def run(use_ros):
    pipeline = VlaPipeline(use_ros=use_ros)
    # ...
```

## Action Server Implementation

For real robots, implement ROS 2 action servers:

### PickObject Server

```python
# src/simulation/action_servers/pick_object_server.py

import rclpy
from rclpy.action import ActionServer
from vla_interfaces.action import PickObject

class PickObjectServer:
    def __init__(self):
        self.node = rclpy.create_node('pick_object_server')
        self.server = ActionServer(
            self.node,
            PickObject,
            '/pick_object',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        """Execute pick action on real robot."""
        feedback = PickObject.Feedback()
        result = PickObject.Result()

        # Phase 1: Approach
        feedback.progress_percent = 0.3
        feedback.current_phase = "approaching"
        goal_handle.publish_feedback(feedback)
        await self.move_to_object(goal_handle.request.object_id)

        # Phase 2: Grasp
        feedback.progress_percent = 0.8
        feedback.current_phase = "grasping"
        goal_handle.publish_feedback(feedback)
        await self.close_gripper(goal_handle.request.grasp_type)

        # Phase 3: Lift
        feedback.progress_percent = 1.0
        feedback.current_phase = "lifting"
        goal_handle.publish_feedback(feedback)
        await self.lift_object()

        result.success = True
        result.error_message = ""
        goal_handle.succeed()
        return result
```

## Gazebo Integration

### Launch Simulation

```python
# src/simulation/launch/humanoid_sim.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', 'household_world.sdf'],
            output='screen'
        ),

        # Start action servers
        Node(
            package='vla_simulation',
            executable='pick_object_server',
            name='pick_object_server'
        ),
        Node(
            package='vla_simulation',
            executable='navigate_server',
            name='navigate_server'
        ),
    ])
```

### Run with ROS 2

```bash
# Terminal 1: Launch simulation
ros2 launch vla_simulation humanoid_sim.launch.py

# Terminal 2: Run VLA with ROS backend
python -m src.cli.vla_cli run --use-ros
```

## Migration Checklist

To add ROS 2 to an existing Pure Python VLA system:

- [ ] Install ROS 2 Humble
- [ ] Build `vla_interfaces` workspace
- [ ] Implement `RosActionService` class
- [ ] Create action server nodes for each action type
- [ ] Add `--use-ros` CLI flag
- [ ] Update `.env` with ROS configuration
- [ ] Test with Gazebo simulation
- [ ] Deploy to physical robot

## Hybrid Approach

You can mix Pure Python and ROS 2:

```python
class HybridActionService(IActionService):
    def __init__(self):
        self.python_service = ActionService()
        self.ros_service = RosActionService()

    async def execute_action(self, action, feedback_callback):
        # Use ROS for hardware-dependent actions
        if action.action_type in [ActionType.PICK, ActionType.PLACE]:
            return await self.ros_service.execute_action(action, feedback_callback)

        # Use Python for software-only actions
        else:
            return await self.python_service.execute_action(action, feedback_callback)
```

## Performance Comparison

| Aspect | Pure Python | ROS 2 |
|--------|-------------|-------|
| Setup Time | < 1 minute | 30-60 minutes |
| Platform Support | Windows/Mac/Linux | Linux only |
| Dependencies | Python packages | System packages + Python |
| Real-time Control | Simulated | Actual hardware |
| Message Latency | 0ms (in-process) | 1-5ms (IPC) |
| Multi-robot | Not designed for | Built-in support |

## When to Migrate

Consider migrating from Pure Python to ROS 2 when:

1. **Hardware Integration**: Need to control real motors/sensors
2. **Physics Accuracy**: Require Gazebo/Isaac Sim simulation
3. **Team Collaboration**: Working with ROS 2 roboticists
4. **Production Deployment**: Moving beyond prototyping
5. **Ecosystem Access**: Need ROS 2 packages (Nav2, MoveIt, etc.)

## Conclusion

The VLA Integration MVP's Pure Python approach is **not a limitation** - it's a deliberate design choice that:
- Eliminates setup friction
- Enables cross-platform development
- Simplifies debugging and testing
- Maintains clear upgrade path to ROS 2

ROS 2 remains available as a powerful extension for production robot deployment, but is **optional** for the core voice-to-action pipeline functionality.
