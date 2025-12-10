"""
Execution Monitor for tracking and logging plan execution.

Coordinates action execution with state tracking and error handling.
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from ..contracts.interfaces import IExecutionMonitor, IActionService
from ..models.action_plan import ActionPlan
from ..models.execution_log import ExecutionLog
from ..models.execution_state import ExecutionError, ExecutionState, ExecutionStatus
from ..utils.config import get_config
from ..utils.errors import ExecutionError as VlaExecutionError
from ..utils.logging_config import PipelineLogger

from .action_service import ActionService

logger = PipelineLogger(stage="execution")


class ExecutionMonitor(IExecutionMonitor):
    """
    Monitors and coordinates action plan execution.

    Features:
        - Step-by-step execution with progress tracking
        - Real-time feedback logging
        - Pause/resume/cancel support
        - Execution trace generation
        - Persistent logging to JSON files
    """

    def __init__(self, action_service: Optional[IActionService] = None):
        """
        Initialize execution monitor.

        Args:
            action_service: Action service instance (None = create default)
        """
        self.config = get_config()
        self.action_service = action_service or ActionService()

        self._current_state: Optional[ExecutionState] = None
        self._current_log: Optional[ExecutionLog] = None
        self._pause_event = asyncio.Event()
        self._cancel_requested = False

        # Set pause event (not paused by default)
        self._pause_event.set()

        logger.info("ExecutionMonitor initialized")

    async def execute_plan(
        self, plan: ActionPlan, state_callback: Optional[Callable] = None
    ) -> ExecutionState:
        """
        Execute action plan and monitor progress.

        Args:
            plan: Plan to execute
            state_callback: Optional callback for state updates

        Returns:
            Final execution state

        Raises:
            VlaExecutionError: If execution fails critically
        """
        logger.start("plan_execution", plan_id=str(plan.plan_id), num_steps=len(plan.steps))
        start_time = datetime.utcnow()

        # Initialize state and log
        self._current_state = ExecutionState(plan_id=plan.plan_id)
        self._current_state.start_execution()

        self._current_log = ExecutionLog(
            plan_id=plan.plan_id,
            voice_command_text="",  # Will be set by pipeline
            parsed_intent_summary="",  # Will be set by pipeline
            plan_steps_summary=[
                f"{step.action_type.value} via {step.ros_action_name}"
                for step in plan.steps
            ],
            execution_trace=[],
            final_status=ExecutionStatus.RUNNING,
            total_duration=0.0,
        )

        self._cancel_requested = False

        try:
            # Connect to action service if needed
            if not self.action_service.is_connected():
                await self.action_service.connect()

            # Execute each step
            for step_index, action in enumerate(plan.steps):
                # Check for cancellation
                if self._cancel_requested:
                    self._current_state.mark_cancelled()
                    break

                # Wait if paused
                await self._pause_event.wait()

                # Update state
                self._current_state.current_step_index = step_index
                self._current_state.current_action_id = action.action_id

                # Log step start
                self._current_log.add_trace_entry(
                    event_type="ACTION_START",
                    step_index=step_index,
                    message=f"Starting {action.action_type.value}",
                    data={"ros_action": action.ros_action_name},
                )

                logger.info(
                    f"Executing step {step_index + 1}/{len(plan.steps)}",
                    action_type=action.action_type.value,
                )

                # Execute action
                try:
                    success, error_msg = await self.action_service.execute_action(
                        action,
                        feedback_callback=lambda feedback: self._handle_feedback(
                            step_index, feedback
                        ),
                    )

                    if success:
                        # Mark step complete
                        self._current_state.complete_step(action.action_id)
                        self._current_state.update_progress(len(plan.steps))

                        self._current_log.add_trace_entry(
                            event_type="ACTION_COMPLETE",
                            step_index=step_index,
                            message=f"Completed {action.action_type.value}",
                        )

                        logger.info(
                            f"Step {step_index + 1} completed",
                            progress=self._current_state.progress_percent,
                        )

                    else:
                        # Action failed
                        error = ExecutionError(
                            timestamp=datetime.utcnow(),
                            step_index=step_index,
                            error_code="ACTION_FAILED",
                            message=error_msg or "Unknown error",
                            recoverable=action.retry_on_failure,
                        )

                        self._current_state.mark_failed(error)

                        self._current_log.add_trace_entry(
                            event_type="ERROR",
                            step_index=step_index,
                            message=error_msg or "Action failed",
                        )

                        logger.error(
                            "plan_execution",
                            Exception(error_msg),
                            step_index=step_index,
                        )

                        break

                except Exception as e:
                    # Unexpected error
                    error = ExecutionError(
                        timestamp=datetime.utcnow(),
                        step_index=step_index,
                        error_code="EXECUTION_ERROR",
                        message=str(e),
                        recoverable=False,
                    )

                    self._current_state.mark_failed(error)
                    logger.error("plan_execution", e, step_index=step_index)
                    break

                # Call state callback
                if state_callback:
                    await asyncio.get_event_loop().run_in_executor(
                        None, state_callback, self._current_state
                    )

            # Check final status
            if (
                self._current_state.status == ExecutionStatus.RUNNING
                and len(self._current_state.completed_steps) == len(plan.steps)
            ):
                self._current_state.mark_completed()

            # Finalize log
            self._current_log.final_status = self._current_state.status
            self._current_log.total_duration = self._current_state.get_duration() or 0.0

            # Save log to file
            if self.config.log_execution_traces:
                await self._save_execution_log()

            duration_ms = (datetime.utcnow() - start_time).total_seconds() * 1000
            logger.complete(
                "plan_execution",
                duration_ms=duration_ms,
                final_status=self._current_state.status.value,
                completed_steps=len(self._current_state.completed_steps),
            )

            return self._current_state

        except Exception as e:
            logger.error("plan_execution", e)
            raise VlaExecutionError(f"Plan execution failed: {e}", plan_id=str(plan.plan_id))

    async def pause_execution(self) -> bool:
        """Pause current execution."""
        if self._current_state and self._current_state.status == ExecutionStatus.RUNNING:
            self._pause_event.clear()
            self._current_state.pause()
            logger.info("Execution paused")
            return True
        return False

    async def resume_execution(self) -> bool:
        """Resume paused execution."""
        if self._current_state and self._current_state.status == ExecutionStatus.PAUSED:
            self._pause_event.set()
            self._current_state.resume()
            logger.info("Execution resumed")
            return True
        return False

    async def cancel_execution(self) -> bool:
        """Cancel current execution."""
        if self._current_state and self._current_state.status in [
            ExecutionStatus.RUNNING,
            ExecutionStatus.PAUSED,
        ]:
            self._cancel_requested = True
            self._pause_event.set()  # Unpause to allow cancellation

            # Cancel current action if any
            if self._current_state.current_action_id:
                await self.action_service.cancel_action(str(self._current_state.current_action_id))

            logger.info("Execution cancelled")
            return True
        return False

    def get_current_state(self) -> Optional[ExecutionState]:
        """Get current execution state."""
        return self._current_state

    def get_current_log(self) -> Optional[ExecutionLog]:
        """Get current execution log."""
        return self._current_log

    def _handle_feedback(self, step_index: int, feedback: dict) -> None:
        """
        Handle action feedback.

        Args:
            step_index: Current step index
            feedback: Feedback data from action
        """
        if self._current_log:
            self._current_log.add_trace_entry(
                event_type="FEEDBACK",
                step_index=step_index,
                message=feedback.get("status_message", ""),
                data=feedback,
            )

    async def _save_execution_log(self) -> None:
        """Save execution log to JSON file."""
        if not self._current_log:
            return

        try:
            log_path = Path(
                self._current_log.get_log_file_path(str(self.config.log_dir))
            )

            # Create directory
            log_path.parent.mkdir(parents=True, exist_ok=True)

            # Write JSON
            with open(log_path, "w", encoding="utf-8") as f:
                json.dump(self._current_log.model_dump(), f, indent=2, default=str)

            logger.info("Execution log saved", path=str(log_path))

        except Exception as e:
            logger.error("Failed to save execution log", error=str(e))


# Example usage
if __name__ == "__main__":
    import sys
    from uuid import uuid4

    from ..models import ActionType
    from ..models.robot_action import RobotAction

    async def test_execution_monitor():
        """Test execution monitor."""
        monitor = ExecutionMonitor()

        # Create test plan
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.NAVIGATE,
                    ros_action_name="/navigate_to_point",
                    goal_message={"target_pose": {"x": 1.0, "y": 0.5}},
                    timeout=2.0,
                ),
                RobotAction(
                    action_type=ActionType.PICK,
                    ros_action_name="/pick_object",
                    goal_message={"object_id": "red_block_1"},
                    timeout=2.0,
                ),
            ],
            preconditions=["Robot ready"],
            expected_outcomes=["Object picked"],
            validated=True,
        )

        print("Executing plan...")

        # Execute
        final_state = await monitor.execute_plan(plan)

        print(f"\nFinal status: {final_state.status.value}")
        print(f"Progress: {final_state.progress_percent:.1f}%")
        print(f"Completed steps: {len(final_state.completed_steps)}/{len(plan.steps)}")

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_execution_monitor())