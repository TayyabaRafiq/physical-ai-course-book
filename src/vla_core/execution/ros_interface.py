"""
ROS 2 Interface for action execution.

Provides async ROS 2 action client with timeout and cancellation support.
Supports both mock mode (for testing without ROS) and real ROS 2 action clients.
"""

import asyncio
import threading
from typing import Callable, Optional, Tuple, Dict, Any

from ..contracts.interfaces import IRosInterface
from ..models.robot_action import RobotAction, ActionType
from ..utils.config import get_config
from ..utils.errors import ExecutionError, RosInterfaceError
from ..utils.logging_config import get_logger

logger = get_logger(__name__)

# Import ROS 2 libraries only when needed (not in mock mode)
try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action.client import ClientGoalHandle, GoalStatus
    from action_msgs.msg import GoalStatus as GoalStatusMsg

    # Import custom action types
    from vla_interfaces.action import PickObject, PlaceObject, NavigateToPoint, InspectObject
    from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning("rclpy or vla_interfaces not available - only mock mode will work")


class VLAActionClientNode(Node if ROS2_AVAILABLE else object):
    """ROS 2 node that manages action clients for VLA actions."""

    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 libraries not available")

        super().__init__('vla_action_client')

        # Create action clients for each action type
        self._action_clients: Dict[str, ActionClient] = {
            '/pick_object': ActionClient(self, PickObject, '/pick_object'),
            '/place_object': ActionClient(self, PlaceObject, '/place_object'),
            '/navigate_to_point': ActionClient(self, NavigateToPoint, '/navigate_to_point'),
            '/inspect_object': ActionClient(self, InspectObject, '/inspect_object'),
        }

        self.get_logger().info('VLA Action Client Node initialized')

    def get_action_client(self, action_name: str) -> Optional[ActionClient]:
        """Get action client by action name."""
        return self._action_clients.get(action_name)


class RosInterface(IRosInterface):
    """
    ROS 2 action client interface.

    Supports both mock mode (for testing without ROS) and real ROS 2 action clients.

    Features:
        - Async action execution
        - Feedback callbacks
        - Action cancellation
        - Timeout handling
        - Automatic ROS node lifecycle management
    """

    def __init__(self, use_mock: bool = True):
        """
        Initialize ROS interface.

        Args:
            use_mock: Use mock implementation (True for testing without ROS)
        """
        self.config = get_config()
        self.use_mock = use_mock

        self._connected = False
        self._current_action_id: Optional[str] = None

        # ROS 2 specific attributes
        self._node: Optional['VLAActionClientNode'] = None
        self._executor: Optional['MultiThreadedExecutor'] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._current_goal_handle: Optional['ClientGoalHandle'] = None

        logger.info("RosInterface initialized", use_mock=use_mock)

    async def connect(self) -> None:
        """Connect to ROS 2."""
        if self.use_mock:
            # Mock connection
            self._connected = True
            logger.info("Mock ROS connection established")
        else:
            # Real ROS 2 connection
            if not ROS2_AVAILABLE:
                raise RosInterfaceError("ROS 2 libraries not available - cannot use real mode")

            try:
                # Initialize rclpy if not already initialized
                if not rclpy.ok():
                    rclpy.init()

                # Create node and executor
                self._node = VLAActionClientNode()
                self._executor = MultiThreadedExecutor()
                self._executor.add_node(self._node)

                # Start spinning in background thread
                self._spin_thread = threading.Thread(
                    target=self._executor.spin,
                    daemon=True
                )
                self._spin_thread.start()

                self._connected = True
                logger.info("ROS 2 connection established")

            except Exception as e:
                raise RosInterfaceError(f"Failed to connect to ROS 2: {e}")

    async def execute_action(
        self, action: RobotAction, feedback_callback: Optional[Callable] = None
    ) -> Tuple[bool, str]:
        """
        Execute a single robot action.

        Args:
            action: Action to execute
            feedback_callback: Optional callback for progress updates

        Returns:
            Tuple of (success, error_message)

        Raises:
            ExecutionError: If action execution fails
        """
        if not self._connected:
            await self.connect()

        self._current_action_id = str(action.action_id)

        logger.info(
            "Executing action",
            action_id=str(action.action_id),
            action_type=action.action_type.value,
            ros_action=action.ros_action_name,
        )

        try:
            if self.use_mock:
                # Mock execution with simulated delay
                return await self._mock_execute_action(action, feedback_callback)
            else:
                # Real ROS execution
                return await self._real_execute_action(action, feedback_callback)

        except Exception as e:
            logger.error("Action execution failed", action_id=str(action.action_id), error=str(e))
            raise ExecutionError(
                f"Action execution failed: {e}",
                action_id=str(action.action_id),
                recoverable=action.retry_on_failure,
            )

        finally:
            self._current_action_id = None

    async def cancel_action(self, action_id: str) -> bool:
        """
        Cancel an in-progress action.

        Args:
            action_id: ID of action to cancel

        Returns:
            True if successfully cancelled
        """
        if self._current_action_id != action_id:
            logger.warning("Cannot cancel - action not current", action_id=action_id)
            return False

        logger.info("Cancelling action", action_id=action_id)

        if self.use_mock:
            # Mock cancellation
            self._current_action_id = None
            return True
        else:
            # Real ROS cancellation
            return await self._cancel_current_goal()

    async def _cancel_current_goal(self) -> bool:
        """Cancel the currently executing goal."""
        if not self._current_goal_handle:
            return False

        try:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            loop = asyncio.get_event_loop()
            cancel_response = await loop.run_in_executor(None, self._wait_for_future, cancel_future, 5.0)
            return True
        except Exception as e:
            logger.error(f"Failed to cancel goal: {e}")
            return False

    def is_connected(self) -> bool:
        """Check if connected to ROS."""
        return self._connected

    async def _mock_execute_action(
        self, action: RobotAction, feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """
        Mock action execution with simulated progress.

        Args:
            action: Action to execute
            feedback_callback: Feedback callback

        Returns:
            Tuple of (success, error_message)
        """
        # Simulate execution with progress updates
        steps = 10
        step_duration = min(action.timeout / steps, 0.5)

        for i in range(steps):
            # Check if cancelled
            if self._current_action_id != str(action.action_id):
                return (False, "Action cancelled")

            # Simulate progress
            progress = (i + 1) / steps * 100.0

            # Send feedback
            if feedback_callback:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    feedback_callback,
                    {
                        "progress_percent": progress,
                        "current_phase": f"step_{i+1}",
                        "status_message": f"Executing {action.action_type.value}",
                    },
                )

            await asyncio.sleep(step_duration)

        # Simulate success
        logger.info(
            "Mock action completed",
            action_id=str(action.action_id),
            action_type=action.action_type.value,
        )

        return (True, "")

    async def _real_execute_action(
        self, action: RobotAction, feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """
        Real ROS 2 action execution.

        Steps:
        1. Get ActionClient for the action type
        2. Create goal message from action.goal_message
        3. Wait for action server
        4. Send goal and wait for acceptance
        5. Wait for result with timeout
        6. Call feedback_callback with progress updates
        7. Return (success, error_message)
        """
        from .ros_action_helpers import (
            create_pick_object_goal,
            create_place_object_goal,
            create_navigate_to_point_goal,
            create_inspect_object_goal,
            extract_feedback_data
        )

        if not self._node:
            raise RosInterfaceError("ROS node not initialized")

        # Get action client
        action_client = self._node.get_action_client(action.ros_action_name)
        if not action_client:
            return (False, f"No action client for {action.ros_action_name}")

        # Wait for action server
        server_available = action_client.wait_for_server(timeout_sec=5.0)
        if not server_available:
            return (False, f"Action server {action.ros_action_name} not available")

        # Create goal message based on action type
        try:
            if action.ros_action_name == '/pick_object':
                goal_msg = create_pick_object_goal(action)
            elif action.ros_action_name == '/place_object':
                goal_msg = create_place_object_goal(action)
            elif action.ros_action_name == '/navigate_to_point':
                goal_msg = create_navigate_to_point_goal(action)
            elif action.ros_action_name == '/inspect_object':
                goal_msg = create_inspect_object_goal(action)
            else:
                return (False, f"Unknown action type: {action.ros_action_name}")
        except Exception as e:
            return (False, f"Failed to create goal message: {e}")

        # Define feedback callback wrapper
        def ros_feedback_callback(feedback_msg):
            if feedback_callback:
                feedback_data = extract_feedback_data(feedback_msg.feedback)
                try:
                    feedback_callback(feedback_data)
                except Exception as e:
                    logger.warning(f"Feedback callback error: {e}")

        # Send goal
        send_goal_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=ros_feedback_callback
        )

        # Wait for goal acceptance (convert Future to async)
        loop = asyncio.get_event_loop()
        try:
            goal_handle = await loop.run_in_executor(None, self._wait_for_future, send_goal_future, 10.0)
        except TimeoutError:
            return (False, "Goal send timeout")

        if not goal_handle.accepted:
            return (False, "Goal rejected by action server")

        self._current_goal_handle = goal_handle

        # Wait for result
        result_future = goal_handle.get_result_async()
        try:
            result = await loop.run_in_executor(None, self._wait_for_future, result_future, action.timeout)
        except TimeoutError:
            # Try to cancel on timeout
            await self._cancel_current_goal()
            return (False, f"Action timeout after {action.timeout}s")
        finally:
            self._current_goal_handle = None

        # Check result status
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            result_data = result.result
            if hasattr(result_data, 'success') and not result_data.success:
                error_msg = getattr(result_data, 'error_message', 'Unknown error')
                return (False, error_msg)
            return (True, "")
        elif result.status == GoalStatus.STATUS_CANCELED:
            return (False, "Action was canceled")
        elif result.status == GoalStatus.STATUS_ABORTED:
            error_msg = getattr(result.result, 'error_message', 'Action aborted')
            return (False, error_msg)
        else:
            return (False, f"Action failed with status: {result.status}")

    def _wait_for_future(self, future, timeout: float):
        """Wait for a Future with timeout (blocking)."""
        import time
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                raise TimeoutError()
            time.sleep(0.01)
        return future.result()

    async def disconnect(self) -> None:
        """Disconnect from ROS."""
        if self._connected:
            if not self.use_mock:
                # Shutdown ROS node and executor
                if self._executor:
                    self._executor.shutdown(timeout_sec=2.0)

                if self._node:
                    self._node.destroy_node()

                # Note: We don't call rclpy.shutdown() as it might be shared
                # In a real application, manage rclpy lifecycle at app level

                logger.info("ROS node and executor shut down")

            self._connected = False
            logger.info("ROS interface disconnected")


# Example usage
if __name__ == "__main__":
    import sys
    from uuid import uuid4

    from ..models import ActionType

    async def test_ros_interface():
        """Test ROS interface."""
        interface = RosInterface(use_mock=True)

        # Connect
        await interface.connect()
        print(f"Connected: {interface.is_connected()}")

        # Create test action
        action = RobotAction(
            action_id=uuid4(),
            action_type=ActionType.PICK,
            ros_action_name="/pick_object",
            goal_message={"object_id": "red_block_1", "grasp_type": "top"},
            timeout=3.0,
        )

        # Execute with feedback
        def feedback_cb(feedback):
            print(f"  Progress: {feedback['progress_percent']:.1f}%")

        print("\nExecuting action...")
        success, error = await interface.execute_action(action, feedback_cb)

        print(f"\nSuccess: {success}")
        if error:
            print(f"Error: {error}")

        # Disconnect
        await interface.disconnect()

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_ros_interface())