"""
Error hierarchy for VLA Integration
Defines custom exceptions for different pipeline stages
"""

from typing import Any, Dict, Optional


class VlaError(Exception):
    """Base exception for all VLA-specific errors."""

    def __init__(
        self,
        message: str,
        error_code: Optional[str] = None,
        context: Optional[Dict[str, Any]] = None,
        recoverable: bool = False
    ):
        super().__init__(message)
        self.message = message
        self.error_code = error_code or self.__class__.__name__.upper()
        self.context = context or {}
        self.recoverable = recoverable

    def __str__(self) -> str:
        parts = [f"[{self.error_code}] {self.message}"]
        if self.context:
            parts.append(f"Context: {self.context}")
        if self.recoverable:
            parts.append("(Recoverable)")
        return " | ".join(parts)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "error_type": self.__class__.__name__,
            "error_code": self.error_code,
            "message": self.message,
            "context": self.context,
            "recoverable": self.recoverable
        }


class TranscriptionError(VlaError):
    """Errors during audio capture or speech-to-text transcription."""

    def __init__(self, message: str, confidence: Optional[float] = None, **kwargs):
        context = kwargs.pop("context", {})
        if confidence is not None:
            context["confidence"] = confidence
        super().__init__(message, context=context, **kwargs)


class AudioCaptureError(VlaError):
    """Errors during microphone audio capture."""

    def __init__(self, message: str, device_index: Optional[int] = None, **kwargs):
        context = kwargs.pop("context", {})
        if device_index is not None:
            context["device_index"] = device_index
        super().__init__(message, context=context, **kwargs)


class ParsingError(VlaError):
    """Errors during intent parsing from transcribed text."""

    def __init__(self, message: str, command_text: Optional[str] = None, **kwargs):
        context = kwargs.pop("context", {})
        if command_text:
            context["command_text"] = command_text
        super().__init__(message, context=context, **kwargs)


class PlanningError(VlaError):
    """Errors during action plan generation."""

    def __init__(self, message: str, intent_summary: Optional[str] = None, **kwargs):
        context = kwargs.pop("context", {})
        if intent_summary:
            context["intent_summary"] = intent_summary
        super().__init__(message, context=context, **kwargs)


class ValidationError(VlaError):
    """Errors during plan safety validation."""

    def __init__(self, message: str, validation_failures: Optional[list] = None, **kwargs):
        context = kwargs.pop("context", {})
        if validation_failures:
            context["validation_failures"] = validation_failures
        super().__init__(message, context=context, **kwargs)


class ExecutionError(VlaError):
    """Errors during robot action execution."""

    def __init__(self, message: str, action_id: Optional[str] = None, step_index: Optional[int] = None, **kwargs):
        context = kwargs.pop("context", {})
        if action_id:
            context["action_id"] = action_id
        if step_index is not None:
            context["step_index"] = step_index
        super().__init__(message, context=context, **kwargs)


class RosInterfaceError(VlaError):
    """Errors in ROS 2 communication layer."""

    def __init__(self, message: str, action_name: Optional[str] = None, **kwargs):
        context = kwargs.pop("context", {})
        if action_name:
            context["action_name"] = action_name
        super().__init__(message, context=context, **kwargs)


class ConfigurationError(VlaError):
    """Errors in configuration loading or validation."""

    def __init__(self, message: str, config_key: Optional[str] = None, **kwargs):
        context = kwargs.pop("context", {})
        if config_key:
            context["config_key"] = config_key
        super().__init__(message, context=context, recoverable=False, **kwargs)


class StateError(VlaError):
    """Errors in state management or invalid state transitions."""

    def __init__(self, message: str, current_state: Optional[str] = None, attempted_state: Optional[str] = None, **kwargs):
        context = kwargs.pop("context", {})
        if current_state:
            context["current_state"] = current_state
        if attempted_state:
            context["attempted_state"] = attempted_state
        super().__init__(message, context=context, **kwargs)


def is_recoverable_error(error: Exception) -> bool:
    """Check if an error is recoverable."""
    if isinstance(error, VlaError):
        return error.recoverable
    return False


def get_error_summary(error: Exception) -> Dict[str, Any]:
    """Get structured summary of error for logging."""
    if isinstance(error, VlaError):
        return error.to_dict()
    return {
        "error_type": type(error).__name__,
        "message": str(error),
        "recoverable": False
    }
