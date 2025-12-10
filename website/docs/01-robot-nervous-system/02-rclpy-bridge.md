# Python-ROS Bridge (rclpy)

## Overview

**rclpy** is the official Python client library for ROS 2, providing a Pythonic interface to ROS 2's middleware. It allows you to create nodes, publish/subscribe to topics, provide services, and control robot behaviors using familiar Python syntax.

## Why Python for Robotics?

| Advantage | Benefit |
|-----------|---------|
| **Rapid Prototyping** | Test ideas quickly without compilation |
| **Rich Ecosystem** | NumPy, OpenCV, PyTorch, scikit-learn |
| **Readable Code** | Easier to maintain and collaborate |
| **Integration** | Seamless with AI/ML frameworks |
| **Learning Curve** | Lower barrier for new roboticists |

## Creating Your First Node

### Minimal Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it**:
```bash
python3 minimal_publisher.py
```

### Minimal Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Lifecycle

### Initialization Pattern

```python
import rclpy
from rclpy.node import Node

def main():
    # 1. Initialize rclpy
    rclpy.init()

    # 2. Create node instance
    node = MyRobotNode()

    try:
        # 3. Spin (process callbacks)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Cleanup
        node.destroy_node()
        rclpy.shutdown()
```

### Multi-Threaded Executor

For handling multiple callbacks concurrently:

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)

try:
    executor.spin()
finally:
    executor.shutdown()
```

## Publishers and Subscribers

### Publishing with QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create custom QoS profile
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

self.publisher = self.create_publisher(
    String,
    'reliable_topic',
    qos_profile
)
```

### Subscriber with Callback Groups

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Create callback groups for parallel processing
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()

        # Sensor subscription (runs in parallel)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
            callback_group=self.sensor_group
        )

        # Control subscription (separate thread)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10,
            callback_group=self.control_group
        )
```

## Services

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
class ClientNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.cli.call_async(req)
        return future

# Usage
node = ClientNode()
future = node.send_request(5, 3)
rclpy.spin_until_future_complete(node, future)
result = future.result()
print(f'Result: {result.sum}')
```

## Actions

### Action Server

```python
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Generate Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Send feedback
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        # Return result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

### Action Client

```python
from rclpy.action import ActionClient

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
```

## Parameters

### Declaring and Using Parameters

```python
class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Declare parameters with defaults
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('robot_name', 'atlas')

        # Get parameter values
        self.max_vel = self.get_parameter('max_velocity').value
        self.use_lidar = self.get_parameter('use_lidar').value
        self.name = self.get_parameter('robot_name').value

        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity':
                self.max_vel = param.value
                self.get_logger().info(f'Updated max_velocity to {param.value}')
        return SetParametersResult(successful=True)
```

### Loading from YAML

**config.yaml**:
```yaml
robot_node:
  ros__parameters:
    max_velocity: 0.8
    use_lidar: true
    robot_name: "optimus"
```

**Launch with parameters**:
```bash
ros2 run my_package robot_node --ros-args --params-file config.yaml
```

## Logging

### Log Levels

```python
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

### Structured Logging

```python
self.get_logger().info(
    f'Robot position: x={x:.2f}, y={y:.2f}, theta={theta:.2f}'
)
```

## Timers

### Periodic Callbacks

```python
class PeriodicNode(Node):
    def __init__(self):
        super().__init__('periodic_node')

        # Create 10 Hz timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'Timer fired: {self.count}')
```

### One-Shot Timer

```python
# Cancel after first execution
def timer_callback(self):
    self.get_logger().info('One-shot timer')
    self.timer.cancel()
```

## TF2 (Transforms)

### Broadcasting Transforms

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_base'

        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.0

        t.transform.rotation.w = 1.0  # No rotation

        self.tf_broadcaster.sendTransform(t)
```

### Listening to Transforms

```python
from tf2_ros import TransformListener, Buffer

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'world',
                'robot_base',
                rclpy.time.Time()
            )
            self.get_logger().info(f'Transform: {trans}')
        except Exception as e:
            self.get_logger().warn(f'Could not lookup transform: {e}')
```

## Integration with VLA Pipeline

### ROS 2 Action Service Backend

```python
from src.vla_core.contracts.action_service import IActionService
from rclpy.action import ActionClient
from vla_interfaces.action import ExecuteRobotAction

class RosActionService(IActionService):
    def __init__(self, node: Node):
        self.node = node
        self.action_client = ActionClient(
            node,
            ExecuteRobotAction,
            'execute_robot_action'
        )

    async def execute_action(self, action, feedback_callback):
        # Convert VLA action to ROS action
        goal = ExecuteRobotAction.Goal()
        goal.action_type = action.action_type.value
        goal.parameters = json.dumps(action.goal_message)

        # Send goal with feedback
        goal_handle = await self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: feedback_callback(fb.feedback)
        )

        # Wait for result
        result = await goal_handle.get_result_async()
        return (result.status == GoalStatus.SUCCEEDED, result.error_message)
```

## Best Practices

1. **Node Naming**: Use descriptive, unique node names
2. **Topic Namespaces**: Organize topics hierarchically (`/robot1/sensors/camera`)
3. **Parameter Validation**: Validate parameter values in callbacks
4. **Resource Cleanup**: Always call `destroy_node()` and `shutdown()`
5. **Type Hints**: Use Python type annotations for clarity
6. **Async Patterns**: Use `async`/`await` for non-blocking operations

## Testing with pytest

```python
import pytest
import rclpy
from my_package.my_node import MyNode

@pytest.fixture
def node():
    rclpy.init()
    node = MyNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_publisher(node):
    # Test publishing logic
    assert node.publisher_ is not None
```

## Next Steps

- **[Chapter 3: URDF for Humanoids](./03-urdf-humanoids)**: Model robot kinematics
- **[Module 2: Digital Twin Simulation](../02-digital-twin-simulation/01-simulation-engines)**: Integrate with Gazebo

## Additional Resources

- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Python Examples](https://github.com/ros2/examples/tree/humble/rclpy)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
