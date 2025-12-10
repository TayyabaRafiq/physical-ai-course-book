---
sidebar_position: 4
---

# ROS 2 Action Definitions

ROS 2 action interface specifications for robot control.

## Action Interface Location

All action definitions are in: `vla_ws/src/vla_interfaces/action/`

## PickObject.action

Grasp an object with the gripper.

```
# Goal
string object_id                    # Unique object identifier
geometry_msgs/Vector3 approach_offset  # Offset for pre-grasp pose (meters)
string grasp_type                   # "top" | "side" | "pinch"
float32 max_force                   # Maximum gripper force (Newtons)

---
# Result
bool success                        # True if object grasped
string error_message                # Error description if failed
geometry_msgs/Pose final_grasp_pose # Final gripper pose

---
# Feedback
string current_phase                # Current execution phase
float32 progress_percent            # Progress 0.0-100.0
```

**Phases**:
1. "Planning approach" - Computing grasp pose
2. "Moving to pre-grasp" - Approaching object
3. "Closing gripper" - Grasping
4. "Verifying grasp" - Checking success

**Example Goal**:
```python
goal_msg = PickObject.Goal()
goal_msg.object_id = "red_block_1"
goal_msg.approach_offset.x = 0.0
goal_msg.approach_offset.y = 0.0
goal_msg.approach_offset.z = 0.1  # 10cm above
goal_msg.grasp_type = "top"
goal_msg.max_force = 30.0
```

## PlaceObject.action

Release object at target location.

```
# Goal
geometry_msgs/Pose target_pose      # Target placement pose
string placement_type               # "gentle" | "drop"
float32 release_height              # Height above surface (meters)
float32 retreat_distance            # Distance to retreat after placement (meters)

---
# Result
bool success                        # True if placed successfully
string error_message                # Error description if failed
geometry_msgs/Pose final_object_pose # Final object pose

---
# Feedback
string current_phase                # Current execution phase
float32 progress_percent            # Progress 0.0-100.0
```

**Phases**:
1. "Moving to placement pose" - Approaching target
2. "Opening gripper" - Releasing object
3. "Retreating" - Moving away
4. "Verifying placement" - Checking stability

**Example Goal**:
```python
goal_msg = PlaceObject.Goal()
goal_msg.target_pose.position.x = 1.5
goal_msg.target_pose.position.y = 0.3
goal_msg.target_pose.position.z = 0.8
goal_msg.target_pose.orientation.w = 1.0
goal_msg.placement_type = "gentle"
goal_msg.release_height = 0.05  # 5cm
goal_msg.retreat_distance = 0.1  # 10cm
```

## NavigateToPoint.action

Navigate mobile base to target pose.

```
# Goal
geometry_msgs/Pose target_pose      # Target pose (x, y, theta)
float32 max_velocity                # Maximum velocity (m/s)
float32 position_tolerance          # Acceptable position error (meters)
float32 orientation_tolerance       # Acceptable orientation error (radians)

---
# Result
bool success                        # True if reached target
string error_message                # Error description if failed
geometry_msgs/Pose final_pose       # Actual final pose
float32 position_error              # Final position error (meters)
float32 orientation_error           # Final orientation error (radians)

---
# Feedback
string current_phase                # Current execution phase
float32 progress_percent            # Progress 0.0-100.0
float32 distance_remaining          # Distance to target (meters)
geometry_msgs/Pose current_pose     # Current robot pose
```

**Phases**:
1. "Planning path" - Computing trajectory
2. "Following path" - Navigating
3. "Final positioning" - Fine alignment

**Example Goal**:
```python
goal_msg = NavigateToPoint.Goal()
goal_msg.target_pose.position.x = 2.0
goal_msg.target_pose.position.y = 1.0
goal_msg.target_pose.orientation.z = 0.707  # 90 degrees
goal_msg.target_pose.orientation.w = 0.707
goal_msg.max_velocity = 0.3
goal_msg.position_tolerance = 0.05  # 5cm
goal_msg.orientation_tolerance = 0.1  # ~5.7 degrees
```

## InspectObject.action

Visual inspection of objects or areas.

```
# Goal
string object_query                 # Object description or "all"
string detection_area               # Area to inspect (e.g., "table", "shelf")
bool find_all                       # Find all matching objects
float32 min_confidence              # Minimum detection confidence

---
# Result
bool success                        # True if inspection completed
string error_message                # Error description if failed
DetectedObject[] detected_objects   # List of detected objects
int32 object_count                  # Number of objects found

---
# Feedback
string current_phase                # Current execution phase
float32 progress_percent            # Progress 0.0-100.0
int32 objects_detected_so_far       # Running count
```

**DetectedObject Message**:
```
string object_id                    # Assigned unique ID
string object_type                  # Detected type
string color                        # Detected color
geometry_msgs/Pose pose             # Object pose
float32 confidence                  # Detection confidence 0.0-1.0
```

**Phases**:
1. "Positioning camera" - Moving to viewing pose
2. "Detecting objects" - Running perception
3. "Verifying detections" - Filtering results

**Example Goal**:
```python
goal_msg = InspectObject.Goal()
goal_msg.object_query = "red block"
goal_msg.detection_area = "table"
goal_msg.find_all = False
goal_msg.min_confidence = 0.7
```

## Building Action Interfaces

### Build Commands

```bash
cd vla_ws

# Build all interfaces
colcon build --packages-select vla_interfaces

# Source workspace
source install/setup.bash

# Verify actions are available
ros2 interface list | grep vla_interfaces
```

**Expected output**:
```
vla_interfaces/action/PickObject
vla_interfaces/action/PlaceObject
vla_interfaces/action/NavigateToPoint
vla_interfaces/action/InspectObject
```

### Using in Python

```python
from vla_interfaces.action import PickObject, PlaceObject, NavigateToPoint

# Create action client
from rclpy.action import ActionClient

pick_client = ActionClient(node, PickObject, '/pick_object')

# Send goal
goal_msg = PickObject.Goal()
goal_msg.object_id = "red_block_1"
goal_msg.grasp_type = "top"

send_goal_future = pick_client.send_goal_async(goal_msg)
```

## Action Server Implementation

Action servers must be implemented for each action type.

**Example skeleton** (`src/ros_action_servers/pick_object_server.py`):

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from vla_interfaces.action import PickObject

class PickObjectServer(Node):
    def __init__(self):
        super().__init__('pick_object_server')
        self._action_server = ActionServer(
            self,
            PickObject,
            '/pick_object',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing PICK: {goal_handle.request.object_id}')

        feedback_msg = PickObject.Feedback()

        # Phase 1: Planning
        feedback_msg.current_phase = "Planning approach"
        feedback_msg.progress_percent = 25.0
        goal_handle.publish_feedback(feedback_msg)

        # ... robot control code ...

        # Phase 4: Verification
        feedback_msg.current_phase = "Verifying grasp"
        feedback_msg.progress_percent = 100.0
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = PickObject.Result()
        result.success = True
        result.error_message = ""
        return result

def main():
    rclpy.init()
    server = PickObjectServer()
    rclpy.spin(server)
```

## Testing Actions

### Test with Command Line

```bash
# Send test goal
ros2 action send_goal /pick_object vla_interfaces/action/PickObject "{object_id: 'test_block', grasp_type: 'top', max_force: 30.0}" --feedback

# Expected output:
# Waiting for an action server to become available...
# Sending goal...
# Feedback: current_phase='Planning approach' progress_percent=25.0
# Feedback: current_phase='Moving to pre-grasp' progress_percent=50.0
# Feedback: current_phase='Closing gripper' progress_percent=75.0
# Feedback: current_phase='Verifying grasp' progress_percent=100.0
# Result: success=True error_message=''
```

## Next Steps

- [API Overview](overview)
- [Data Models](data-models)
- [Execution Layer](../architecture/execution-layer)