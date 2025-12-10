---
sidebar_position: 5
---

# Developer Guide

Guide for extending and customizing the VLA Integration system.

## Adding New Action Types

### 1. Define Action Type Enum

Edit `src/vla_core/models/enums.py`:

```python
class ActionType(str, Enum):
    PICK = "pick"
    PLACE = "place"
    NAVIGATE = "navigate"
    INSPECT = "inspect"
    HANDOVER = "handover"
    PUSH = "push"  # NEW
```

### 2. Create ROS 2 Action Definition

Create `vla_ws/src/vla_interfaces/action/PushObject.action`:

```
# Goal
string object_id
geometry_msgs/Vector3 push_direction
float32 push_distance
float32 max_force

---
# Result
bool success
string error_message
geometry_msgs/Pose final_object_pose

---
# Feedback
string current_phase
float32 progress_percent
```

Rebuild interfaces:

```bash
cd vla_ws
colcon build --packages-select vla_interfaces
source install/setup.bash
```

### 3. Update Intent Parser Prompt

Edit `src/vla_core/cognition/prompts/intent_parser_prompt.md`:

```markdown
## Available Action Types

- **PICK**: Grasp an object
- **PLACE**: Release object at location
- **NAVIGATE**: Move to position
- **INSPECT**: Visual examination
- **HANDOVER**: Transfer object to person
- **PUSH**: Push object in direction  # NEW

## PUSH Action Parameters

- `push_direction`: Vector3 direction to push
- `push_distance`: Distance to push (meters)
- `max_force`: Maximum push force (Newtons)
```

### 4. Update Action Planner Prompt

Edit `src/vla_core/cognition/prompts/action_planner_prompt.md`:

Add PUSH examples and planning guidelines.

### 5. Update Plan Validator

Edit `src/vla_core/cognition/plan_validator.py`:

```python
def _validate_step(self, step: RobotAction, step_index: int) -> List[str]:
    errors = []

    # Existing validations...

    # Add PUSH-specific validation
    if step.action_type == ActionType.PUSH:
        if "push_direction" not in step.goal_message:
            errors.append(f"Step {step_index}: PUSH missing push_direction")

    return errors
```

### 6. Implement ROS 2 Action Server

Create `src/ros_action_servers/push_object_server.py`:

```python
from rclpy.action import ActionServer
from vla_interfaces.action import PushObject

class PushObjectServer(Node):
    def __init__(self):
        super().__init__('push_object_server')
        self._action_server = ActionServer(
            self,
            PushObject,
            '/push_object',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        # Implement push logic
        result = PushObject.Result()
        result.success = True
        return result
```

## Custom LLM Prompts

### Modifying Intent Parser

```python
# Create custom prompt file
custom_prompt = """
You are a specialized intent parser for industrial robot tasks.
Focus on precision and safety.

Available Actions: PICK, PLACE, INSPECT

Output JSON format:
{
  "action_type": "...",
  "target_objects": [...],
  "parameters": {...}
}
"""

# Save to file
with open("prompts/custom_intent_parser.md", "w") as f:
    f.write(custom_prompt)

# Use in IntentParser
parser = IntentParser(config)
with open("prompts/custom_intent_parser.md") as f:
    parser.system_prompt = f.read()
```

### Templating System

Use variables in prompts:

```markdown
# Intent Parser

Robot Capabilities:
- DOF: {robot_dof}
- Max Reach: {max_reach}m
- Gripper Type: {gripper_type}

Available Actions: {action_types}
```

Load with formatting:

```python
prompt_template = load_template("intent_parser_prompt.md")
prompt = prompt_template.format(
    robot_dof=7,
    max_reach=0.8,
    gripper_type="parallel",
    action_types=", ".join([a.value for a in ActionType])
)
```

## Alternative LLM Providers

### Using Local LLaMA

```python
from langchain_community.llms import Ollama

class LocalIntentParser(IIntentParser):
    def __init__(self, config):
        self.llm = Ollama(model="llama2")

    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        prompt = f"{self.system_prompt}\n\nUser: {command.transcribed_text}"
        response = await self.llm.ainvoke(prompt)

        # Parse JSON response
        data = json.loads(response)
        return ParsedIntent(**data)
```

Use in pipeline:

```python
pipeline = VlaPipeline()
pipeline.intent_parser = LocalIntentParser(config)
```

### Using Anthropic Claude

```python
import anthropic

class ClaudeIntentParser(IIntentParser):
    def __init__(self, config):
        self.client = anthropic.AsyncAnthropic(api_key=config.anthropic_api_key)

    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        message = await self.client.messages.create(
            model="claude-3-opus-20240229",
            max_tokens=1024,
            system=self.system_prompt,
            messages=[{"role": "user", "content": command.transcribed_text}]
        )

        data = json.loads(message.content[0].text)
        return ParsedIntent(**data)
```

## Custom Voice Recognition

### Using Google Speech API

```python
from google.cloud import speech

class GoogleTranscriber(ITranscriber):
    def __init__(self):
        self.client = speech.SpeechClient()

    async def transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        audio = speech.RecognitionAudio(content=audio_data)
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code="en-US",
        )

        response = await self.client.recognize(config=config, audio=audio)

        if response.results:
            result = response.results[0]
            text = result.alternatives[0].transcript
            confidence = result.alternatives[0].confidence
            return (text, confidence)

        return ("", 0.0)
```

## Hardware Integration

### Custom Robot Platform

Implement `IRosInterface` for your robot:

```python
class CustomRobotInterface(IRosInterface):
    def __init__(self, robot_ip: str):
        self.robot = YourRobotSDK.connect(robot_ip)

    async def execute_action(
        self,
        action: RobotAction,
        feedback_callback: Optional[Callable] = None
    ) -> Tuple[bool, str]:
        if action.action_type == ActionType.PICK:
            return await self._execute_pick(action, feedback_callback)
        elif action.action_type == ActionType.PLACE:
            return await self._execute_place(action, feedback_callback)
        # ... handle other actions

    async def _execute_pick(self, action, callback):
        object_id = action.goal_message["object_id"]

        # Use your robot's API
        success = await self.robot.grasp_object(object_id)

        if callback:
            callback({"progress_percent": 100.0})

        return (success, "" if success else "Grasp failed")
```

## Testing Strategies

### Unit Testing Components

```python
import pytest
from unittest.mock import AsyncMock

@pytest.mark.asyncio
async def test_custom_intent_parser():
    parser = CustomIntentParser(config)

    command = VoiceCommand(
        audio_buffer=b"test",
        transcribed_text="Pick up the block",
        confidence=0.95
    )

    intent = await parser.parse(command)

    assert intent.action_type == ActionType.PICK
    assert len(intent.target_objects) > 0
```

### Integration Testing

```python
@pytest.mark.integration
async def test_full_pipeline_with_custom_components():
    pipeline = VlaPipeline()

    # Inject custom components
    pipeline.intent_parser = CustomIntentParser(config)
    pipeline.ros_interface = MockRobotInterface()

    log = await pipeline.process_voice_command(timeout=5.0)

    assert log.final_status == ExecutionStatus.COMPLETED
```

### Mocking ROS 2

```python
class MockActionClient:
    async def send_goal_async(self, goal):
        # Simulate action execution
        await asyncio.sleep(0.1)
        return MockGoalHandle(accepted=True)

@pytest.fixture
def mock_ros_interface(monkeypatch):
    interface = RosInterface(config)
    interface._action_clients = {
        "/pick_object": MockActionClient(),
        "/place_object": MockActionClient(),
    }
    return interface
```

## Performance Optimization

### Caching LLM Responses

```python
from functools import lru_cache

class CachedLLMClient:
    @lru_cache(maxsize=100)
    def _get_cached_response(self, prompt_hash: str) -> str:
        return self._call_llm(prompt_hash)

    async def chat_completion(self, system_prompt: str, user_message: str) -> str:
        prompt_hash = hashlib.md5(
            f"{system_prompt}{user_message}".encode()
        ).hexdigest()

        return self._get_cached_response(prompt_hash)
```

### Parallel Action Execution

For independent actions:

```python
async def execute_parallel_actions(actions: List[RobotAction]):
    tasks = [
        ros_interface.execute_action(action)
        for action in actions
    ]

    results = await asyncio.gather(*tasks)
    return results
```

### GPU Memory Management

```python
import torch

# Clear cache periodically
torch.cuda.empty_cache()

# Use mixed precision
with torch.cuda.amp.autocast():
    result = model.transcribe(audio)
```

## Logging and Monitoring

### Custom Log Handler

```python
import structlog

class MetricsLogger:
    def __init__(self):
        self.logger = structlog.get_logger()

    def log_latency(self, operation: str, duration_ms: float):
        self.logger.info(
            "operation_latency",
            operation=operation,
            duration_ms=duration_ms,
            metric_type="latency"
        )

    def log_error(self, operation: str, error: Exception):
        self.logger.error(
            "operation_error",
            operation=operation,
            error_type=type(error).__name__,
            error_message=str(error)
        )
```

### Prometheus Metrics

```python
from prometheus_client import Counter, Histogram

action_counter = Counter('vla_actions_total', 'Total actions executed', ['action_type'])
action_duration = Histogram('vla_action_duration_seconds', 'Action execution time')

@action_duration.time()
async def execute_action(action):
    result = await ros_interface.execute_action(action)
    action_counter.labels(action_type=action.action_type.value).inc()
    return result
```

## Deployment

### Docker Container

```dockerfile
FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    portaudio19-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install -r requirements.txt

# Copy application
COPY src/ /app/src/
WORKDIR /app

# Run pipeline
CMD ["python", "-m", "src.cli.vla_cli", "run", "--loop"]
```

### Environment-Specific Configs

```bash
# .env.production
ROS_USE_MOCK=false
WHISPER_MODEL=medium
OPENAI_MODEL=gpt-4-turbo-preview
LOG_LEVEL=INFO

# .env.development
ROS_USE_MOCK=true
WHISPER_MODEL=base
OPENAI_MODEL=gpt-3.5-turbo
LOG_LEVEL=DEBUG
```

## Security Considerations

### API Key Management

```python
# Use environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Or secrets management
from azure.keyvault.secrets import SecretClient
client = SecretClient(vault_url="...", credential=...)
api_key = client.get_secret("openai-api-key").value
```

### Input Validation

```python
from pydantic import field_validator

class SafeActionPlan(ActionPlan):
    @field_validator('steps')
    def validate_safe_actions(cls, v):
        for step in v:
            if step.constraints.max_force > SAFETY_LIMIT:
                raise ValueError("Force exceeds safety limit")
        return v
```

### Sandboxing

Run in restricted environment:

```python
import resource

# Limit memory
resource.setrlimit(resource.RLIMIT_AS, (4 * 1024**3, 4 * 1024**3))  # 4GB

# Limit CPU time
resource.setrlimit(resource.RLIMIT_CPU, (300, 300))  # 5 minutes
```

## Contributing

### Code Style

```bash
# Format code
black src/ tests/
isort src/ tests/

# Type check
mypy src/

# Lint
flake8 src/ tests/
```

### Pull Request Checklist

- [ ] Tests added/updated
- [ ] Documentation updated
- [ ] Type hints added
- [ ] Code formatted with black
- [ ] No linting errors
- [ ] Changelog updated

## Next Steps

- [Architecture Overview](architecture/overview)
- [API Reference](api/overview)
- [Troubleshooting](tutorials/troubleshooting)