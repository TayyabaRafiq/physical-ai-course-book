---
sidebar_position: 2
---

# Data Models

Complete reference for all Pydantic data models.

## Voice Layer Models

### VoiceCommand

Voice input with transcription and metadata.

**Location**: `src/vla_core/models/voice_command.py`

```python
class VoiceCommand(BaseModel):
    id: UUID = Field(default_factory=uuid4)
    audio_buffer: bytes
    transcribed_text: str
    confidence: float = Field(ge=0.0, le=1.0)
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    def is_high_confidence(self, threshold: float = 0.7) -> bool:
        """Check if transcription confidence meets threshold."""
        return self.confidence >= threshold
```

**Fields**:
- `id`: Unique identifier
- `audio_buffer`: Raw audio bytes (WAV format)
- `transcribed_text`: Whisper transcription output
- `confidence`: Confidence score from 0.0-1.0
- `timestamp`: UTC timestamp of capture

**Example**:
```python
command = VoiceCommand(
    audio_buffer=b"\x00\x01...",
    transcribed_text="Pick up the red block",
    confidence=0.95
)
```

## Cognition Layer Models

### ParsedIntent

Structured command interpretation from LLM.

**Location**: `src/vla_core/models/parsed_intent.py`

```python
class ObjectReference(BaseModel):
    name: str
    type: str
    color: Optional[str] = None
    size: Optional[str] = None
    location: Optional[str] = None
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)

class ParsedIntent(BaseModel):
    command_id: UUID
    action_type: ActionType
    target_objects: List[ObjectReference] = Field(max_length=5)
    parameters: Dict[str, Any] = Field(default_factory=dict)
    ambiguities: List[str] = Field(default_factory=list)
    requires_clarification: bool = False

    def validate_parameters_for_action(self) -> bool:
        """Validate required parameters for action type."""
        if self.action_type == ActionType.PICK:
            if not self.target_objects:
                self.ambiguities.append("No target object specified")
                self.requires_clarification = True
                return False
        return True
```

**Fields**:
- `command_id`: Reference to VoiceCommand
- `action_type`: PICK | PLACE | NAVIGATE | INSPECT | HANDOVER
- `target_objects`: List of ObjectReference
- `parameters`: Action-specific parameters
- `ambiguities`: Detected ambiguities
- `requires_clarification`: Whether user input needed

**Example**:
```python
intent = ParsedIntent(
    command_id=uuid4(),
    action_type=ActionType.PICK,
    target_objects=[
        ObjectReference(
            name="red block",
            type="block",
            color="red",
            confidence=0.92
        )
    ],
    parameters={"grasp_type": "top", "approach_offset": {"z": 0.1}}
)
```

### ActionPlan

Multi-step action sequence with validation.

**Location**: `src/vla_core/models/action_plan.py`

```python
class ActionPlan(BaseModel):
    plan_id: UUID = Field(default_factory=uuid4)
    intent_id: UUID
    steps: List[RobotAction] = Field(min_length=1, max_length=50)
    preconditions: List[str] = Field(default_factory=list)
    expected_outcomes: List[str] = Field(default_factory=list)
    estimated_duration: float = Field(ge=0.0)
    is_validated: bool = False
    validation_errors: List[str] = Field(default_factory=list)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    def mark_validated(self) -> None:
        """Mark plan as validated."""
        self.is_validated = True
        self.validation_errors = []

    def add_validation_error(self, error: str) -> None:
        """Add validation error."""
        self.validation_errors.append(error)
        self.is_validated = False
```

**Fields**:
- `plan_id`: Unique plan identifier
- `intent_id`: Reference to ParsedIntent
- `steps`: List of RobotAction (1-50 steps)
- `preconditions`: Required initial conditions
- `expected_outcomes`: Expected results
- `estimated_duration`: Total duration estimate (seconds)
- `is_validated`: Validation status
- `validation_errors`: List of validation failures

### RobotAction

Single atomic robot action.

**Location**: `src/vla_core/models/robot_action.py`

```python
class RobotActionConstraints(BaseModel):
    max_force: float = Field(default=50.0, ge=0.0, le=200.0)
    max_velocity: float = Field(default=0.5, ge=0.0, le=2.0)
    max_acceleration: float = Field(default=1.0, ge=0.0, le=5.0)
    workspace_bounds: dict = Field(default_factory=dict)

class RobotAction(BaseModel):
    action_id: UUID = Field(default_factory=uuid4)
    action_type: ActionType
    ros_action_name: str
    goal_message: dict
    timeout: float = Field(default=30.0, gt=0.0, le=300.0)
    constraints: RobotActionConstraints = Field(default_factory=RobotActionConstraints)
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

**Fields**:
- `action_id`: Unique action identifier
- `action_type`: ActionType enum
- `ros_action_name`: ROS 2 action server topic
- `goal_message`: Action-specific goal parameters
- `timeout`: Maximum execution time (seconds)
- `constraints`: Safety constraints

**Example**:
```python
action = RobotAction(
    action_type=ActionType.PICK,
    ros_action_name="/pick_object",
    goal_message={
        "object_id": "red_block_1",
        "grasp_type": "top",
        "max_force": 30.0
    },
    timeout=8.0
)
```

## Execution Layer Models

### ExecutionState

Real-time execution status.

**Location**: `src/vla_core/models/execution_state.py`

```python
class ExecutionError(BaseModel):
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    step_index: int
    error_code: str
    message: str
    recoverable: bool = False

class ExecutionState(BaseModel):
    plan_id: UUID
    status: ExecutionStatus = ExecutionStatus.IDLE
    current_step_index: int = Field(default=0, ge=0)
    current_action_id: Optional[UUID] = None
    completed_steps: List[UUID] = Field(default_factory=list)
    progress_percent: float = Field(default=0.0, ge=0.0, le=100.0)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    last_error: Optional[ExecutionError] = None

    def start_execution(self) -> None:
        """Mark execution start."""
        self.status = ExecutionStatus.RUNNING
        self.started_at = datetime.utcnow()

    def complete_step(self, action_id: UUID) -> None:
        """Mark step as completed."""
        self.completed_steps.append(action_id)

    def update_progress(self, total_steps: int) -> None:
        """Update progress percentage."""
        self.progress_percent = (len(self.completed_steps) / total_steps) * 100.0

    def mark_completed(self) -> None:
        """Mark execution complete."""
        self.status = ExecutionStatus.COMPLETED
        self.completed_at = datetime.utcnow()
        self.progress_percent = 100.0

    def mark_failed(self, error: ExecutionError) -> None:
        """Mark execution failed."""
        self.status = ExecutionStatus.FAILED
        self.last_error = error
        self.completed_at = datetime.utcnow()

    def get_duration(self) -> float:
        """Get execution duration in seconds."""
        if not self.started_at:
            return 0.0
        end_time = self.completed_at or datetime.utcnow()
        return (end_time - self.started_at).total_seconds()
```

**Fields**:
- `plan_id`: Reference to ActionPlan
- `status`: IDLE | RUNNING | PAUSED | COMPLETED | FAILED | CANCELLED
- `current_step_index`: Current step (0-indexed)
- `current_action_id`: Currently executing action
- `completed_steps`: List of completed action IDs
- `progress_percent`: Overall progress (0.0-100.0)
- `started_at`: Execution start time
- `completed_at`: Execution end time
- `last_error`: Most recent error (if any)

### ExecutionLog

Persistent audit trail.

**Location**: `src/vla_core/models/execution_log.py`

```python
class TraceEntry(BaseModel):
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    event_type: str
    step_index: int
    message: str
    metadata: Dict[str, Any] = Field(default_factory=dict)

class ExecutionLog(BaseModel):
    log_id: UUID = Field(default_factory=uuid4)
    plan_id: UUID
    voice_command_text: str
    parsed_intent_summary: str
    plan_steps_summary: List[str]
    execution_trace: List[TraceEntry] = Field(default_factory=list)
    final_status: Optional[ExecutionStatus] = None
    total_duration: Optional[float] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    def add_trace_entry(
        self,
        event_type: str,
        step_index: int,
        message: str,
        **metadata
    ) -> None:
        """Add trace entry."""
        entry = TraceEntry(
            event_type=event_type,
            step_index=step_index,
            message=message,
            metadata=metadata
        )
        self.execution_trace.append(entry)
```

**Fields**:
- `log_id`: Unique log identifier
- `plan_id`: Reference to ActionPlan
- `voice_command_text`: Original voice command
- `parsed_intent_summary`: Intent summary
- `plan_steps_summary`: List of action types
- `execution_trace`: Timestamped events
- `final_status`: Final execution status
- `total_duration`: Total duration (seconds)
- `created_at`: Log creation time

**Example Trace Entry**:
```python
log.add_trace_entry(
    event_type="ACTION_START",
    step_index=0,
    message="Starting navigation",
    target_location="table"
)
```

## Enumerations

### ActionType

```python
class ActionType(str, Enum):
    PICK = "pick"
    PLACE = "place"
    NAVIGATE = "navigate"
    INSPECT = "inspect"
    HANDOVER = "handover"
```

### ExecutionStatus

```python
class ExecutionStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
```

## Validation Rules

All models use Pydantic validators:

```python
from pydantic import field_validator

class ActionPlan(BaseModel):
    steps: List[RobotAction]

    @field_validator('steps')
    def validate_steps(cls, v):
        if not v:
            raise ValueError("Plan must have at least one step")
        if len(v) > 50:
            raise ValueError("Plan exceeds maximum steps")
        return v
```

## Serialization

Models support JSON serialization:

```python
# To JSON
command_json = command.model_dump()
command_str = command.model_dump_json(indent=2)

# From JSON
command = VoiceCommand.model_validate(command_dict)
command = VoiceCommand.model_validate_json(command_str)
```

## Next Steps

- [Interfaces](interfaces) - Interface specifications
- [ROS Actions](ros-actions) - ROS 2 action definitions
- [API Overview](overview) - API introduction