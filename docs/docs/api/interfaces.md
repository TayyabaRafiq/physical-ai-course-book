---
sidebar_position: 3
---

# Interface Specifications

Abstract base classes for dependency inversion.

## Voice Layer Interfaces

### IAudioCapture

```python
class IAudioCapture(ABC):
    @abstractmethod
    async def start_capture(self, device_index: Optional[int] = None) -> None:
        """Start capturing audio from microphone."""
        pass

    @abstractmethod
    async def stop_capture(self) -> None:
        """Stop capturing audio."""
        pass

    @abstractmethod
    async def get_audio_stream(self) -> AsyncIterator[bytes]:
        """Get asynchronous stream of audio chunks."""
        pass

    @abstractmethod
    async def record_fixed_duration(self, timeout: float) -> bytes:
        """Record audio for fixed duration."""
        pass
```

### ITranscriber

```python
class ITranscriber(ABC):
    @abstractmethod
    async def transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        """
        Transcribe audio to text.

        Returns:
            Tuple of (text, confidence)
        """
        pass
```

### IVoiceInterface

```python
class IVoiceInterface(ABC):
    @abstractmethod
    async def listen(self, timeout: Optional[float] = None) -> VoiceCommand:
        """Listen for voice command with retry logic."""
        pass

    @abstractmethod
    async def initialize(self) -> None:
        """Initialize voice interface."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Cleanup resources."""
        pass
```

## Cognition Layer Interfaces

### IIntentParser

```python
class IIntentParser(ABC):
    @abstractmethod
    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        """Parse voice command to structured intent."""
        pass
```

### IActionPlanner

```python
class IActionPlanner(ABC):
    @abstractmethod
    async def generate_plan(self, intent: ParsedIntent) -> ActionPlan:
        """Generate multi-step action plan from intent."""
        pass

    @abstractmethod
    async def replan(
        self,
        intent: ParsedIntent,
        failed_step: int,
        failure_reason: str
    ) -> ActionPlan:
        """Generate alternative plan after failure."""
        pass
```

### IPlanValidator

```python
class IPlanValidator(ABC):
    @abstractmethod
    async def validate(self, plan: ActionPlan) -> Tuple[bool, List[str]]:
        """
        Validate action plan.

        Returns:
            Tuple of (is_valid, errors)
        """
        pass
```

## Execution Layer Interfaces

### IRosInterface

```python
class IRosInterface(ABC):
    @abstractmethod
    async def initialize(self) -> None:
        """Initialize ROS 2 connection."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown ROS 2 connection."""
        pass

    @abstractmethod
    async def execute_action(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable[[dict], None]] = None
    ) -> Tuple[bool, str]:
        """
        Execute robot action via ROS 2.

        Returns:
            Tuple of (success, error_message)
        """
        pass

    @abstractmethod
    async def cancel_action(self) -> bool:
        """Cancel currently executing action."""
        pass
```

### IExecutionMonitor

```python
class IExecutionMonitor(ABC):
    @abstractmethod
    async def execute_plan(
        self,
        plan: ActionPlan,
        state_callback: Optional[Callable[[ExecutionState], None]] = None
    ) -> ExecutionState:
        """Execute action plan with monitoring."""
        pass

    @abstractmethod
    async def pause_execution(self) -> None:
        """Pause execution after current action."""
        pass

    @abstractmethod
    async def resume_execution(self) -> None:
        """Resume paused execution."""
        pass

    @abstractmethod
    async def cancel_execution(self) -> None:
        """Cancel execution."""
        pass

    @abstractmethod
    def get_current_state(self) -> Optional[ExecutionState]:
        """Get current execution state."""
        pass

    @abstractmethod
    def get_current_log(self) -> Optional[ExecutionLog]:
        """Get current execution log."""
        pass
```

## Pipeline Interface

### IVlaPipeline

```python
class IVlaPipeline(ABC):
    @abstractmethod
    async def initialize(self) -> None:
        """Initialize pipeline components."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown pipeline."""
        pass

    @abstractmethod
    async def process_voice_command(
        self,
        timeout: Optional[float] = None,
        state_callback: Optional[Callable[[ExecutionState], None]] = None
    ) -> ExecutionLog:
        """Process complete voice-to-action pipeline."""
        pass
```

## Implementation Example

```python
class CustomIntentParser(IIntentParser):
    def __init__(self, config: VLAConfig):
        self.config = config

    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        # Custom parsing logic
        return ParsedIntent(
            command_id=command.id,
            action_type=ActionType.PICK,
            target_objects=[],
            parameters={}
        )
```

Use in pipeline:

```python
pipeline = VlaPipeline()
pipeline.intent_parser = CustomIntentParser(config)
```

## Next Steps

- [ROS Actions](ros-actions)
- [Data Models](data-models)