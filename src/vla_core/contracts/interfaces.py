"""
Python interface contracts for VLA Integration
Defines abstract base classes for all major components
"""

from abc import ABC, abstractmethod
from typing import Any, AsyncIterator, Dict, List, Optional

from ..models import ActionType, ExecutionStatus
from ..models.voice_command import VoiceCommand
from ..models.parsed_intent import ParsedIntent
from ..models.action_plan import ActionPlan
from ..models.robot_action import RobotAction
from ..models.execution_state import ExecutionState
from ..models.execution_log import ExecutionLog


class IAudioCapture(ABC):
    """Interface for audio capture from microphone."""

    @abstractmethod
    async def start_recording(self) -> None:
        """Start audio capture."""
        pass

    @abstractmethod
    async def stop_recording(self) -> bytes:
        """Stop recording and return audio buffer."""
        pass

    @abstractmethod
    async def get_audio_stream(self) -> AsyncIterator[bytes]:
        """Get continuous audio stream for real-time processing."""
        pass

    @abstractmethod
    def is_recording(self) -> bool:
        """Check if currently recording."""
        pass


class ITranscriber(ABC):
    """Interface for speech-to-text transcription."""

    @abstractmethod
    async def transcribe(self, audio_buffer: bytes) -> tuple[str, float]:
        """
        Transcribe audio to text.
        
        Returns:
            Tuple of (transcribed_text, confidence_score)
        """
        pass

    @abstractmethod
    async def load_model(self) -> None:
        """Load transcription model (e.g., Whisper)."""
        pass

    @abstractmethod
    def is_model_loaded(self) -> bool:
        """Check if model is ready."""
        pass


class IVoiceInterface(ABC):
    """Interface for complete voice input processing."""

    @abstractmethod
    async def listen_for_command(self) -> VoiceCommand:
        """
        Capture and transcribe a voice command.
        
        Returns:
            VoiceCommand with transcribed text and metadata
        """
        pass

    @abstractmethod
    async def start_continuous_listening(self) -> AsyncIterator[VoiceCommand]:
        """Start background listening for continuous commands."""
        pass

    @abstractmethod
    def set_confidence_threshold(self, threshold: float) -> None:
        """Set minimum confidence for accepting transcriptions."""
        pass


class IIntentParser(ABC):
    """Interface for parsing natural language into structured intent."""

    @abstractmethod
    async def parse(self, command_text: str) -> ParsedIntent:
        """
        Parse command text into structured intent.
        
        Args:
            command_text: Transcribed voice command
            
        Returns:
            ParsedIntent with action type, objects, and parameters
        """
        pass

    @abstractmethod
    async def request_clarification(self, ambiguity: str) -> str:
        """Request user clarification for ambiguous command."""
        pass


class IActionPlanner(ABC):
    """Interface for generating action plans from intents."""

    @abstractmethod
    async def generate_plan(self, intent: ParsedIntent) -> ActionPlan:
        """
        Generate action plan from parsed intent.
        
        Args:
            intent: Parsed command intent
            
        Returns:
            ActionPlan with ordered robot actions
        """
        pass

    @abstractmethod
    async def replan(self, failed_step: int, original_plan: ActionPlan) -> ActionPlan:
        """Generate alternative plan after step failure."""
        pass

    @abstractmethod
    def set_max_steps(self, max_steps: int) -> None:
        """Set maximum allowed plan complexity."""
        pass


class IPlanValidator(ABC):
    """Interface for validating action plan safety."""

    @abstractmethod
    async def validate(self, plan: ActionPlan) -> tuple[bool, List[str]]:
        """
        Validate plan against safety constraints.
        
        Returns:
            Tuple of (is_valid, validation_errors)
        """
        pass

    @abstractmethod
    def check_joint_limits(self, action: RobotAction) -> bool:
        """Verify action respects joint limits."""
        pass

    @abstractmethod
    def check_workspace_bounds(self, action: RobotAction) -> bool:
        """Verify target pose is reachable."""
        pass

    @abstractmethod
    def check_collision(self, action: RobotAction) -> bool:
        """Check for potential collisions."""
        pass


class IActionService(ABC):
    """Interface for robot action execution service."""

    @abstractmethod
    async def execute_action(self, action: RobotAction, feedback_callback=None) -> tuple[bool, str]:
        """
        Execute single robot action with optional feedback.

        Args:
            action: Robot action to execute
            feedback_callback: Optional callback for progress updates

        Returns:
            Tuple of (success, error_message)
        """
        pass

    @abstractmethod
    async def cancel_action(self, action_id: str) -> bool:
        """Cancel currently executing action."""
        pass

    @abstractmethod
    async def get_feedback(self, action_id: str) -> Dict[str, Any]:
        """Get real-time feedback from action server."""
        pass

    @abstractmethod
    def is_action_server_available(self, action_name: str) -> bool:
        """Check if ROS action server is online."""
        pass


class IExecutionMonitor(ABC):
    """Interface for monitoring and controlling execution."""

    @abstractmethod
    async def execute_plan(self, plan: ActionPlan) -> ExecutionState:
        """
        Execute complete action plan.
        
        Returns:
            Final ExecutionState after completion/failure
        """
        pass

    @abstractmethod
    async def pause_execution(self) -> None:
        """Pause current execution."""
        pass

    @abstractmethod
    async def resume_execution(self) -> None:
        """Resume paused execution."""
        pass

    @abstractmethod
    async def cancel_execution(self) -> None:
        """Cancel current execution."""
        pass

    @abstractmethod
    def get_current_state(self) -> ExecutionState:
        """Get current execution state."""
        pass

    @abstractmethod
    async def save_execution_log(self, log_path: str) -> None:
        """Save execution log to file."""
        pass


class IVlaPipeline(ABC):
    """Interface for complete voice-to-action pipeline orchestration."""

    @abstractmethod
    async def process_voice_command(self) -> ExecutionLog:
        """
        Process complete voice-to-action workflow.
        
        Returns:
            ExecutionLog with complete trace
        """
        pass

    @abstractmethod
    async def start_interactive_mode(self) -> None:
        """Start continuous command processing loop."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Gracefully shutdown pipeline."""
        pass

    @abstractmethod
    def get_pipeline_status(self) -> Dict[str, Any]:
        """Get current pipeline component statuses."""
        pass
