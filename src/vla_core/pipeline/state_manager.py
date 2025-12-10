"""
StateManager for VLA Pipeline
Manages in-memory state tracking across pipeline stages
"""

import asyncio
from datetime import datetime
from typing import Dict, Optional
from uuid import UUID

from ..models import ExecutionStatus
from ..models.voice_command import VoiceCommand
from ..models.parsed_intent import ParsedIntent
from ..models.action_plan import ActionPlan
from ..models.execution_state import ExecutionState
from ..utils.errors import StateError



class StateManager:
    """
    Manages in-memory state for the VLA pipeline.
    Tracks current command, intent, plan, and execution state.
    Thread-safe for concurrent access.
    """

    def __init__(self):
        """Initialize state manager with empty state."""
        self._lock = asyncio.Lock()
        self._current_command: Optional[VoiceCommand] = None
        self._current_intent: Optional[ParsedIntent] = None
        self._current_plan: Optional[ActionPlan] = None
        self._execution_state: Optional[ExecutionState] = None
        self._history: Dict[UUID, Dict] = {}  # plan_id -> full state snapshot

    async def set_voice_command(self, command: VoiceCommand) -> None:
        """Set current voice command."""
        async with self._lock:
            self._current_command = command

    async def get_voice_command(self) -> Optional[VoiceCommand]:
        """Get current voice command."""
        async with self._lock:
            return self._current_command

    async def set_parsed_intent(self, intent: ParsedIntent) -> None:
        """Set current parsed intent."""
        async with self._lock:
            self._current_intent = intent

    async def get_parsed_intent(self) -> Optional[ParsedIntent]:
        """Get current parsed intent."""
        async with self._lock:
            return self._current_intent

    async def set_action_plan(self, plan: ActionPlan) -> None:
        """Set current action plan."""
        async with self._lock:
            self._current_plan = plan
            # Initialize execution state for this plan
            self._execution_state = ExecutionState(
                plan_id=plan.plan_id,
                status=ExecutionStatus.IDLE
            )

    async def get_action_plan(self) -> Optional[ActionPlan]:
        """Get current action plan."""
        async with self._lock:
            return self._current_plan

    async def get_execution_state(self) -> Optional[ExecutionState]:
        """Get current execution state."""
        async with self._lock:
            return self._execution_state

    async def update_execution_state(
        self,
        status: Optional[ExecutionStatus] = None,
        current_step_index: Optional[int] = None,
        current_action_id: Optional[UUID] = None,
        progress_percent: Optional[float] = None
    ) -> None:
        """
        Update execution state fields.
        
        Args:
            status: New execution status
            current_step_index: Current step being executed
            current_action_id: Current action UUID
            progress_percent: Overall progress percentage
        """
        async with self._lock:
            if self._execution_state is None:
                raise StateError("Cannot update execution state: no active plan")

            # Validate status transition if changing status
            if status is not None:
                if not self._execution_state.can_transition_to(status):
                    raise StateError(
                        f"Invalid status transition from {self._execution_state.status} to {status}",
                        current_state=str(self._execution_state.status),
                        attempted_state=str(status)
                    )
                self._execution_state.status = status

                # Set timestamps for status transitions
                if status == ExecutionStatus.RUNNING and self._execution_state.started_at is None:
                    self._execution_state.started_at = datetime.utcnow()
                elif status.is_terminal() and self._execution_state.completed_at is None:
                    self._execution_state.completed_at = datetime.utcnow()

            # Update other fields if provided
            if current_step_index is not None:
                self._execution_state.current_step_index = current_step_index
            if current_action_id is not None:
                self._execution_state.current_action_id = current_action_id
            if progress_percent is not None:
                self._execution_state.progress_percent = progress_percent

    async def mark_step_completed(self, action_id: UUID) -> None:
        """Mark a step as completed."""
        async with self._lock:
            if self._execution_state is None:
                raise StateError("Cannot mark step completed: no active execution")
            
            if action_id not in self._execution_state.completed_steps:
                self._execution_state.completed_steps.append(action_id)

    async def add_execution_error(
        self,
        step_index: int,
        error_code: str,
        message: str,
        recoverable: bool = False
    ) -> None:
        """Add an error to execution state."""
        async with self._lock:
            if self._execution_state is None:
                raise StateError("Cannot add error: no active execution")
            
            self._execution_state.add_error(
                step_index=step_index,
                error_code=error_code,
                message=message,
                recoverable=recoverable
            )

    async def save_to_history(self) -> None:
        """Save current state snapshot to history."""
        async with self._lock:
            if self._current_plan is None:
                return

            self._history[self._current_plan.plan_id] = {
                "command": self._current_command,
                "intent": self._current_intent,
                "plan": self._current_plan,
                "execution_state": self._execution_state,
                "saved_at": datetime.utcnow()
            }

    async def get_history(self, plan_id: UUID) -> Optional[Dict]:
        """Retrieve historical state for a plan."""
        async with self._lock:
            return self._history.get(plan_id)

    async def clear_current_state(self) -> None:
        """Clear current state (after completion or cancellation)."""
        async with self._lock:
            # Save to history before clearing
            if self._current_plan is not None:
                await self.save_to_history()

            self._current_command = None
            self._current_intent = None
            self._current_plan = None
            self._execution_state = None

    async def get_full_state(self) -> Dict:
        """Get complete current state snapshot."""
        async with self._lock:
            return {
                "command": self._current_command,
                "intent": self._current_intent,
                "plan": self._current_plan,
                "execution_state": self._execution_state
            }

    def is_idle(self) -> bool:
        """Check if state manager has no active work."""
        return (
            self._current_command is None and
            self._current_intent is None and
            self._current_plan is None and
            self._execution_state is None
        )

    def has_active_execution(self) -> bool:
        """Check if there is an active execution in progress."""
        return (
            self._execution_state is not None and
            self._execution_state.status == ExecutionStatus.RUNNING
        )

    # Alias methods for pipeline compatibility
    async def store_voice_command(self, command: VoiceCommand) -> None:
        """Alias for set_voice_command."""
        await self.set_voice_command(command)

    async def store_intent(self, intent: ParsedIntent) -> None:
        """Alias for set_parsed_intent."""
        await self.set_parsed_intent(intent)

    async def store_plan(self, plan: ActionPlan) -> None:
        """Alias for set_action_plan."""
        await self.set_action_plan(plan)
