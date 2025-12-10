"""
Pure Python Action Service for robot action execution.

This replaces ROS 2 as the execution layer with a simple, dependency-free
Python implementation suitable for MVP testing and demonstration.
"""

import asyncio
from typing import Callable, Optional, Tuple, Dict, Any
from uuid import UUID

from ..contracts.interfaces import IActionService
from ..models.robot_action import RobotAction, ActionType
from ..utils.config import get_config
from ..utils.errors import ExecutionError
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class ActionService(IActionService):
    """
    Pure Python action execution service.

    Simulates robot actions with realistic progress updates and timing.
    Suitable for MVP testing without external dependencies.

    Features:
        - Async action execution with simulated progress
        - Feedback callbacks for progress updates
        - Action cancellation support
        - Configurable execution delays
        - Detailed logging
    """

    def __init__(self):
        """Initialize action service."""
        self.config = get_config()
        self._connected = True  # Always "connected" in pure Python mode
        self._current_action_id: Optional[str] = None
        self._cancel_requested = False

        logger.info("ActionService initialized (Pure Python mode)")

    async def connect(self) -> None:
        """Connect to action service (no-op in pure Python mode)."""
        self._connected = True
        logger.info("ActionService ready")

    async def execute_action(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable] = None
    ) -> Tuple[bool, str]:
        """
        Execute a robot action with simulated progress.

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
        self._cancel_requested = False

        logger.info(
            "Executing action",
            action_id=str(action.action_id),
            action_type=action.action_type.value,
        )

        try:
            # Route to specific action handler
            if action.action_type == ActionType.NAVIGATE:
                return await self._execute_navigate(action, feedback_callback)
            elif action.action_type == ActionType.PICK:
                return await self._execute_pick(action, feedback_callback)
            elif action.action_type == ActionType.PLACE:
                return await self._execute_place(action, feedback_callback)
            elif action.action_type == ActionType.INSPECT:
                return await self._execute_inspect(action, feedback_callback)
            elif action.action_type == ActionType.WAIT:
                return await self._execute_wait(action, feedback_callback)
            elif action.action_type == ActionType.STOP:
                return await self._execute_stop(action, feedback_callback)
            else:
                return (False, f"Unknown action type: {action.action_type}")

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
        self._cancel_requested = True
        return True

    def is_connected(self) -> bool:
        """Check if service is ready."""
        return self._connected

    async def disconnect(self) -> None:
        """Disconnect from service."""
        self._connected = False
        logger.info("ActionService disconnected")

    # Action-specific execution methods

    async def _execute_navigate(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """Execute navigation action."""
        goal = action.goal_message
        target = goal.get("target_pose", {})
        x = target.get("x", 0.0)
        y = target.get("y", 0.0)

        # Simulate navigation with progress
        distance = (x**2 + y**2) ** 0.5
        steps = max(5, int(distance * 2))  # More steps for longer distances

        for i in range(steps):
            if self._cancel_requested:
                return (False, "Navigation cancelled")

            progress = (i + 1) / steps * 100.0
            current_x = x * (i + 1) / steps
            current_y = y * (i + 1) / steps

            if feedback_callback:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    feedback_callback,
                    {
                        "progress_percent": progress,
                        "current_phase": f"navigating",
                        "status_message": f"Moving to ({x:.2f}, {y:.2f})",
                        "current_position": {"x": current_x, "y": current_y}
                    }
                )

            await asyncio.sleep(action.timeout / steps)

        logger.info(
            "Navigation completed",
            action_id=str(action.action_id),
            target_x=x,
            target_y=y
        )
        return (True, "")

    async def _execute_pick(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """Execute pick action."""
        goal = action.goal_message
        object_id = goal.get("object_id", "unknown")
        grasp_type = goal.get("grasp_type", "top")

        phases = [
            ("approaching", 0.3),
            ("aligning", 0.5),
            ("grasping", 0.8),
            ("lifting", 1.0)
        ]

        for phase_name, phase_progress in phases:
            if self._cancel_requested:
                return (False, "Pick cancelled")

            if feedback_callback:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    feedback_callback,
                    {
                        "progress_percent": phase_progress * 100,
                        "current_phase": phase_name,
                        "status_message": f"Picking {object_id} ({grasp_type} grasp)",
                        "object_id": object_id
                    }
                )

            await asyncio.sleep(action.timeout / len(phases))

        logger.info(
            "Pick completed",
            action_id=str(action.action_id),
            object_id=object_id,
            grasp_type=grasp_type
        )
        return (True, "")

    async def _execute_place(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """Execute place action."""
        goal = action.goal_message
        pose = goal.get("placement_pose", {})
        surface = goal.get("placement_surface", "surface")

        phases = [
            ("moving_to_target", 0.25),
            ("lowering", 0.5),
            ("releasing", 0.75),
            ("retracting", 1.0)
        ]

        for phase_name, phase_progress in phases:
            if self._cancel_requested:
                return (False, "Place cancelled")

            if feedback_callback:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    feedback_callback,
                    {
                        "progress_percent": phase_progress * 100,
                        "current_phase": phase_name,
                        "status_message": f"Placing object on {surface}",
                        "target_surface": surface
                    }
                )

            await asyncio.sleep(action.timeout / len(phases))

        logger.info(
            "Place completed",
            action_id=str(action.action_id),
            surface=surface
        )
        return (True, "")

    async def _execute_inspect(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """Execute inspect action."""
        goal = action.goal_message
        target_id = goal.get("target_id", "area")
        duration = goal.get("inspection_duration", 2.0)

        steps = max(3, int(duration))

        for i in range(steps):
            if self._cancel_requested:
                return (False, "Inspection cancelled")

            progress = (i + 1) / steps * 100.0

            if feedback_callback:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    feedback_callback,
                    {
                        "progress_percent": progress,
                        "current_phase": "scanning",
                        "status_message": f"Inspecting {target_id}",
                        "target_id": target_id
                    }
                )

            await asyncio.sleep(duration / steps)

        logger.info(
            "Inspection completed",
            action_id=str(action.action_id),
            target_id=target_id
        )
        return (True, "")

    async def _execute_wait(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """Execute wait action."""
        duration = action.timeout
        steps = max(2, int(duration))

        for i in range(steps):
            if self._cancel_requested:
                return (False, "Wait cancelled")

            progress = (i + 1) / steps * 100.0
            remaining = duration * (1 - progress / 100)

            if feedback_callback:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    feedback_callback,
                    {
                        "progress_percent": progress,
                        "current_phase": "waiting",
                        "status_message": f"Waiting ({remaining:.1f}s remaining)",
                    }
                )

            await asyncio.sleep(duration / steps)

        logger.info("Wait completed", action_id=str(action.action_id))
        return (True, "")

    async def _execute_stop(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """Execute stop action."""
        logger.info("Emergency stop", action_id=str(action.action_id))

        if feedback_callback:
            await asyncio.get_event_loop().run_in_executor(
                None,
                feedback_callback,
                {
                    "progress_percent": 100.0,
                    "current_phase": "stopped",
                    "status_message": "Emergency stop executed"
                }
            )

        return (True, "")


# Example usage
if __name__ == "__main__":
    import sys
    from uuid import uuid4

    async def test_action_service():
        """Test action service."""
        service = ActionService()

        # Connect
        await service.connect()
        print(f"Connected: {service.is_connected()}")

        # Create test action
        from ..models.robot_action import RobotAction, ActionConstraints

        action = RobotAction(
            action_id=uuid4(),
            action_type=ActionType.PICK,
            ros_action_name="/pick_object",  # Not used in pure Python mode
            goal_message={"object_id": "red_block_1", "grasp_type": "top"},
            timeout=3.0,
        )

        # Execute with feedback
        def feedback_cb(feedback):
            print(f"  Progress: {feedback['progress_percent']:.1f}% - {feedback.get('current_phase', 'unknown')}")

        print("\nExecuting action...")
        success, error = await service.execute_action(action, feedback_cb)

        print(f"\nSuccess: {success}")
        if error:
            print(f"Error: {error}")

        # Disconnect
        await service.disconnect()

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_action_service())