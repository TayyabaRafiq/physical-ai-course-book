---
sidebar_position: 1
---

# API Overview

Complete API reference for the VLA Integration system.

## Package Structure

```
src/vla_core/
├── models/              # Pydantic data models
│   ├── voice_command.py
│   ├── parsed_intent.py
│   ├── action_plan.py
│   ├── robot_action.py
│   ├── execution_state.py
│   └── execution_log.py
├── voice/               # Voice interface layer
│   ├── audio_capture.py
│   ├── transcription.py
│   └── voice_interface.py
├── cognition/           # Cognitive planning layer
│   ├── intent_parser.py
│   ├── action_planner.py
│   ├── plan_validator.py
│   └── llm_client.py
├── execution/           # Execution layer
│   ├── ros_interface.py
│   └── execution_monitor.py
├── pipeline/            # Pipeline orchestration
│   ├── vla_pipeline.py
│   └── state_manager.py
└── utils/               # Utilities
    ├── config.py
    ├── logging_config.py
    └── errors.py
```

## Core Interfaces

All major components implement abstract base classes for dependency inversion:

```python
from abc import ABC, abstractmethod

class IVoiceInterface(ABC):
    @abstractmethod
    async def listen(self, timeout: Optional[float] = None) -> VoiceCommand:
        """Capture and transcribe voice command."""
        pass

class IIntentParser(ABC):
    @abstractmethod
    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        """Parse voice command to structured intent."""
        pass

class IActionPlanner(ABC):
    @abstractmethod
    async def generate_plan(self, intent: ParsedIntent) -> ActionPlan:
        """Generate multi-step action plan."""
        pass

class IPlanValidator(ABC):
    @abstractmethod
    async def validate(self, plan: ActionPlan) -> Tuple[bool, List[str]]:
        """Validate plan for safety and feasibility."""
        pass

class IRosInterface(ABC):
    @abstractmethod
    async def execute_action(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable] = None
    ) -> Tuple[bool, str]:
        """Execute robot action via ROS 2."""
        pass

class IExecutionMonitor(ABC):
    @abstractmethod
    async def execute_plan(
        self,
        plan: ActionPlan,
        state_callback: Optional[Callable] = None
    ) -> ExecutionState:
        """Execute and monitor action plan."""
        pass

class IVlaPipeline(ABC):
    @abstractmethod
    async def process_voice_command(
        self,
        timeout: Optional[float] = None
    ) -> ExecutionLog:
        """Process complete voice-to-action pipeline."""
        pass
```

## Data Models

All data models use Pydantic v2 for validation:

```python
from pydantic import BaseModel, Field
from uuid import UUID, uuid4
from datetime import datetime

class VoiceCommand(BaseModel):
    """Voice input with transcription."""
    id: UUID = Field(default_factory=uuid4)
    audio_buffer: bytes
    transcribed_text: str
    confidence: float = Field(ge=0.0, le=1.0)
    timestamp: datetime = Field(default_factory=datetime.utcnow)

class ParsedIntent(BaseModel):
    """Structured command interpretation."""
    command_id: UUID
    action_type: ActionType
    target_objects: List[ObjectReference]
    parameters: Dict[str, Any]
    ambiguities: List[str]
    requires_clarification: bool

class ActionPlan(BaseModel):
    """Multi-step action sequence."""
    plan_id: UUID = Field(default_factory=uuid4)
    intent_id: UUID
    steps: List[RobotAction]
    preconditions: List[str]
    expected_outcomes: List[str]
    estimated_duration: float
    is_validated: bool = False
    validation_errors: List[str] = []

class ExecutionState(BaseModel):
    """Real-time execution status."""
    plan_id: UUID
    status: ExecutionStatus
    current_step_index: int
    completed_steps: List[UUID]
    progress_percent: float
    last_error: Optional[ExecutionError]
```

See [Data Models](data-models) for complete reference.

## Enumerations

### ActionType

```python
from enum import Enum

class ActionType(str, Enum):
    """Available robot actions."""
    PICK = "pick"
    PLACE = "place"
    NAVIGATE = "navigate"
    INSPECT = "inspect"
    HANDOVER = "handover"
```

### ExecutionStatus

```python
class ExecutionStatus(str, Enum):
    """Execution state machine states."""
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
```

## Configuration

Configuration via Pydantic settings with `.env` file support:

```python
from pydantic_settings import BaseSettings, SettingsConfigDict

class VLAConfig(BaseSettings):
    """Global configuration."""
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8")

    # OpenAI API
    openai_api_key: str
    openai_model: str = "gpt-4-turbo-preview"
    openai_temperature: float = Field(default=0.7, ge=0.0, le=2.0)

    # Whisper
    whisper_model: str = "medium"
    whisper_device: str = "cuda"
    whisper_confidence_threshold: float = Field(default=0.7, ge=0.0, le=1.0)

    # Safety Limits
    max_plan_steps: int = Field(default=10, ge=1, le=50)
    max_force_threshold: float = Field(default=50.0, ge=0.0)
    max_velocity_threshold: float = Field(default=0.5, ge=0.0)

    # ROS 2
    ros_use_mock: bool = True
    ros_domain_id: int = 0

    # Workspace
    workspace_bounds: dict = Field(
        default={
            "x_min": -1.0, "x_max": 3.0,
            "y_min": -2.0, "y_max": 2.0,
            "z_min": 0.0, "z_max": 2.0
        }
    )
```

## Error Handling

Custom exception hierarchy:

```python
class VlaError(Exception):
    """Base exception for VLA system."""
    def __init__(self, message: str, recoverable: bool = False, **context):
        self.message = message
        self.recoverable = recoverable
        self.context = context
        super().__init__(message)

class TranscriptionError(VlaError):
    """Voice transcription failed."""
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
    """Action execution failed."""
    pass

class RosInterfaceError(VlaError):
    """ROS 2 communication error."""
    pass
```

## Logging

Structured logging with `structlog`:

```python
from src.vla_core.utils.logging_config import get_logger

logger = get_logger(__name__)

logger.info(
    "Action completed",
    action_id=str(action.id),
    action_type=action.action_type.value,
    duration_ms=duration,
    success=True,
)
```

**Output** (JSON format):
```json
{
  "event": "Action completed",
  "timestamp": "2025-12-08T10:30:15Z",
  "level": "info",
  "logger": "vla_core.execution.ros_interface",
  "action_id": "550e8400-e29b-41d4-a716-446655440000",
  "action_type": "pick",
  "duration_ms": 5300,
  "success": true
}
```

## Async/Await Pattern

All I/O-bound operations use async:

```python
async def main():
    # Create pipeline
    pipeline = VlaPipeline()

    # Initialize (async)
    await pipeline.initialize()

    # Process command (async)
    log = await pipeline.process_voice_command(timeout=10.0)

    # Cleanup
    await pipeline.shutdown()

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
```

## Type Annotations

Full type hints throughout:

```python
from typing import Optional, List, Dict, Tuple, Callable, Any, AsyncIterator

async def execute_action(
    action: RobotAction,
    feedback_callback: Optional[Callable[[dict], None]] = None,
    timeout: Optional[float] = None,
) -> Tuple[bool, str]:
    """
    Execute robot action.

    Args:
        action: Robot action to execute
        feedback_callback: Optional callback for progress updates
        timeout: Optional timeout in seconds

    Returns:
        Tuple of (success, error_message)

    Raises:
        RosInterfaceError: If ROS communication fails
        ExecutionError: If action execution fails
    """
    pass
```

## Usage Examples

### Basic Pipeline

```python
from src.vla_core.pipeline.vla_pipeline import VlaPipeline

async def run_pipeline():
    pipeline = VlaPipeline()
    await pipeline.initialize()

    try:
        log = await pipeline.process_voice_command(timeout=10.0)
        print(f"Status: {log.final_status}")
        print(f"Duration: {log.total_duration}s")
    finally:
        await pipeline.shutdown()

asyncio.run(run_pipeline())
```

### Custom Component

```python
from src.vla_core.cognition.interfaces import IIntentParser

class CustomIntentParser(IIntentParser):
    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        # Custom parsing logic
        return ParsedIntent(...)

# Use in pipeline
pipeline = VlaPipeline()
pipeline.intent_parser = CustomIntentParser(config)
```

### State Monitoring

```python
def state_callback(state: ExecutionState):
    print(f"Progress: {state.progress_percent:.1f}%")
    print(f"Status: {state.status.value}")

log = await pipeline.process_voice_command(
    timeout=10.0,
    state_callback=state_callback
)
```

## Testing

### Unit Tests

```python
import pytest
from unittest.mock import AsyncMock, Mock

@pytest.mark.asyncio
async def test_voice_interface():
    interface = VoiceInterface(config)

    # Mock dependencies
    interface.audio_capture.record_fixed_duration = AsyncMock(
        return_value=b"test_audio"
    )
    interface.transcriber.transcribe = AsyncMock(
        return_value=("Test command", 0.95)
    )

    # Test
    command = await interface.listen(timeout=5.0)

    # Assert
    assert command.transcribed_text == "Test command"
    assert command.confidence == 0.95
```

### Integration Tests

```python
@pytest.mark.asyncio
async def test_full_pipeline():
    pipeline = VlaPipeline()

    # Mock all external dependencies
    # ... setup mocks ...

    log = await pipeline.process_voice_command(timeout=5.0)

    assert log.final_status == ExecutionStatus.COMPLETED
```

## Next Steps

- [Data Models](data-models) - Complete model reference
- [Interfaces](interfaces) - Interface specifications
- [ROS Actions](ros-actions) - ROS 2 action definitions