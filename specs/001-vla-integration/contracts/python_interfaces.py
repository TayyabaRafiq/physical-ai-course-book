"""
Python Interface Contracts for VLA Integration

These abstract base classes define the boundaries between pipeline layers.
Implementations must adhere to these interfaces for testability and modularity.
"""

from abc import ABC, abstractmethod
from typing import Optional, AsyncIterator
from dataclasses import dataclass
import numpy as np
from datetime import datetime
from uuid import UUID


# ============================================================================
# Voice Layer Interfaces
# ============================================================================

class IAudioCapture(ABC):
    """Interface for capturing audio from microphone."""

    @abstractmethod
    async def start_listening(self) -> None:
        """Begin capturing audio from default input device."""
        pass

    @abstractmethod
    async def stop_listening(self) -> None:
        """Stop audio capture and release device."""
        pass

    @abstractmethod
    async def get_audio_stream(self) -> AsyncIterator[np.ndarray]:
        """
        Yield audio chunks as they're captured.

        Yields:
            np.ndarray: Audio samples (float32, mono, sample_rate=16000)
        """
        pass


class ITranscriber(ABC):
    """Interface for speech-to-text transcription."""

    @abstractmethod
    async def transcribe(self, audio: np.ndarray) -> tuple[str, float]:
        """
        Convert audio to text.

        Args:
            audio: Audio samples (float32, mono, 16kHz)

        Returns:
            tuple[str, float]: (transcribed_text, confidence_score)

        Raises:
            TranscriptionError: If transcription fails
        """
        pass


@dataclass
class VoiceCommand:
    """Output of voice interface layer."""
    id: UUID
    audio_buffer: bytes
    transcribed_text: str
    confidence: float
    timestamp: datetime
    language: str
    sample_rate: int


class IVoiceInterface(ABC):
    """High-level interface for voice command capture."""

    @abstractmethod
    async def listen_for_command(self, timeout_seconds: float = 30.0) -> VoiceCommand:
        """
        Capture and transcribe a single voice command.

        Args:
            timeout_seconds: Max wait time for voice activity

        Returns:
            VoiceCommand: Captured and transcribed command

        Raises:
            TimeoutError: If no voice activity detected
            TranscriptionError: If speech recognition fails
        """
        pass


# ============================================================================
# Cognition Layer Interfaces
# ============================================================================

@dataclass
class ParsedIntent:
    """Structured command interpretation."""
    command_id: UUID
    action_type: str  # ActionType enum value
    target_objects: list[dict]
    parameters: dict
    ambiguities: list[str]
    requires_clarification: bool


class IIntentParser(ABC):
    """Interface for natural language understanding."""

    @abstractmethod
    async def parse(self, text: str) -> ParsedIntent:
        """
        Extract structured intent from natural language text.

        Args:
            text: Natural language command (e.g., "Pick up the red block")

        Returns:
            ParsedIntent: Structured representation of command

        Raises:
            ParsingError: If LLM call fails or response is malformed
        """
        pass


@dataclass
class ActionPlan:
    """Sequence of robot actions."""
    plan_id: UUID
    intent_id: UUID
    steps: list['RobotAction']
    preconditions: list[str]
    expected_outcomes: list[str]
    estimated_duration: float
    validated: bool
    validation_errors: list[str]


class IActionPlanner(ABC):
    """Interface for cognitive task planning."""

    @abstractmethod
    async def generate_plan(self, intent: ParsedIntent) -> ActionPlan:
        """
        Generate action sequence from parsed intent.

        Args:
            intent: Structured command representation

        Returns:
            ActionPlan: Ordered sequence of robot actions

        Raises:
            PlanningError: If no valid plan can be generated
        """
        pass


@dataclass
class ValidationResult:
    """Result of plan safety validation."""
    valid: bool
    errors: list[str]
    warnings: list[str]


class IPlanValidator(ABC):
    """Interface for safety constraint checking."""

    @abstractmethod
    def validate_plan(self, plan: ActionPlan) -> ValidationResult:
        """
        Check plan against safety constraints.

        Args:
            plan: Action sequence to validate

        Returns:
            ValidationResult: Pass/fail with error details
        """
        pass


# ============================================================================
# Execution Layer Interfaces
# ============================================================================

@dataclass
class RobotAction:
    """Single executable robot action."""
    action_id: UUID
    action_type: str
    ros_action_name: str
    goal_message: dict
    constraints: dict
    timeout: float
    retry_on_failure: bool


@dataclass
class ExecutionState:
    """Real-time execution status."""
    plan_id: UUID
    status: str  # ExecutionStatus enum value
    current_step_index: int
    completed_steps: list[UUID]
    current_action_id: Optional[UUID]
    progress_percent: float
    errors: list[dict]
    started_at: datetime
    completed_at: Optional[datetime]


class IRosInterface(ABC):
    """Interface for ROS 2 action execution."""

    @abstractmethod
    async def execute_action(self, action: RobotAction) -> dict:
        """
        Send action goal to ROS action server and wait for result.

        Args:
            action: Robot action to execute

        Returns:
            dict: ROS action result message

        Raises:
            ActionTimeoutError: If action exceeds timeout
            ActionFailedError: If action server reports failure
        """
        pass

    @abstractmethod
    async def cancel_action(self, action_id: UUID) -> bool:
        """
        Request cancellation of in-progress action.

        Args:
            action_id: ID of action to cancel

        Returns:
            bool: True if cancellation accepted
        """
        pass

    @abstractmethod
    async def get_action_feedback(self, action_id: UUID) -> AsyncIterator[dict]:
        """
        Stream feedback updates from executing action.

        Args:
            action_id: ID of action to monitor

        Yields:
            dict: ROS action feedback message
        """
        pass


class IExecutionMonitor(ABC):
    """Interface for tracking plan execution."""

    @abstractmethod
    async def execute_plan(self, plan: ActionPlan) -> ExecutionState:
        """
        Execute action plan and monitor progress.

        Args:
            plan: Validated action sequence

        Returns:
            ExecutionState: Final execution status

        Raises:
            ExecutionError: If plan execution fails
        """
        pass

    @abstractmethod
    async def pause_execution(self, plan_id: UUID) -> bool:
        """Pause currently executing plan."""
        pass

    @abstractmethod
    async def resume_execution(self, plan_id: UUID) -> bool:
        """Resume paused plan."""
        pass

    @abstractmethod
    def get_execution_state(self, plan_id: UUID) -> ExecutionState:
        """Get current state of plan execution."""
        pass


# ============================================================================
# Pipeline Orchestration Interface
# ============================================================================

class IVlaPipeline(ABC):
    """Top-level interface for end-to-end voice-to-action pipeline."""

    @abstractmethod
    async def process_voice_command(self, timeout: float = 30.0) -> ExecutionState:
        """
        Execute full pipeline: listen → parse → plan → execute.

        Args:
            timeout: Max time to wait for voice input

        Returns:
            ExecutionState: Final execution result

        Raises:
            PipelineError: If any stage fails
        """
        pass

    @abstractmethod
    async def handle_interruption(self, command_text: str) -> None:
        """
        Process real-time interruption command (pause/cancel/stop).

        Args:
            command_text: Interruption command text
        """
        pass


# ============================================================================
# Error Hierarchy
# ============================================================================

class VlaError(Exception):
    """Base exception for VLA system."""
    pass


class TranscriptionError(VlaError):
    """Speech recognition failed."""
    pass


class ParsingError(VlaError):
    """Intent parsing failed."""
    pass


class PlanningError(VlaError):
    """Action planning failed."""
    pass


class ValidationError(VlaError):
    """Plan validation failed."""
    pass


class ExecutionError(VlaError):
    """Plan execution failed."""
    pass


class ActionTimeoutError(ExecutionError):
    """ROS action exceeded timeout."""
    pass


class ActionFailedError(ExecutionError):
    """ROS action server reported failure."""
    pass


class PipelineError(VlaError):
    """Pipeline orchestration failed."""
    pass