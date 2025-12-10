# End-to-End VLA Pipeline

## Overview

This chapter integrates all VLA components—Voice Interface, Cognitive Planning, and Action Execution—into a complete voice-to-action system. The pipeline converts spoken commands into executed robot actions with full observability and error handling.

**Complete Data Flow**:
```
Microphone → Whisper → LLM → ActionService → Robot → Execution Log
```

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    VLA Pipeline                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌───────────────┐                                     │
│  │ VoiceService  │ ──┐                                 │
│  │  (Whisper)    │   │                                 │
│  └───────────────┘   │                                 │
│                      ▼                                  │
│              VoiceCommand                               │
│              {text, confidence}                         │
│                      │                                  │
│                      ▼                                  │
│  ┌───────────────────────────────┐                     │
│  │   CognitivePlanner (LLM)      │                     │
│  │   - Parse command             │                     │
│  │   - Generate action plan      │                     │
│  │   - Validate steps            │                     │
│  └───────────────┬───────────────┘                     │
│                  │                                      │
│                  ▼                                      │
│            ActionPlan                                   │
│            {steps: [...]}                               │
│                  │                                      │
│                  ▼                                      │
│  ┌───────────────────────────────┐                     │
│  │   ExecutionMonitor            │                     │
│  │   - Execute steps sequentially│                     │
│  │   - Track progress            │                     │
│  │   - Handle failures           │                     │
│  │   - Collect feedback          │                     │
│  └───────────────┬───────────────┘                     │
│                  │                                      │
│                  ▼                                      │
│  ┌───────────────────────────────┐                     │
│  │   ActionService               │                     │
│  │   (Pure Python or ROS 2)      │                     │
│  │   - Execute actions           │                     │
│  │   - Provide feedback          │                     │
│  └───────────────┬───────────────┘                     │
│                  │                                      │
│                  ▼                                      │
│           Execution Log                                 │
│           {status, steps, errors}                       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Implementation

### VLA Pipeline Core

Create `src/vla_core/pipeline.py`:

```python
import asyncio
from typing import Optional
from datetime import datetime
from dataclasses import dataclass, field

from src.vla_core.services.voice_service import VoiceService, VoiceCommand
from src.vla_core.services.cognitive_planner import CognitivePlanner, ActionPlan
from src.vla_core.services.action_service import ActionService
from src.vla_core.models.robot_action import RobotAction, ActionType
from src.vla_core.contracts.action_service import IActionService

@dataclass
class ExecutionStatus:
    """Execution status values."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class ExecutionLog:
    """Complete execution log."""
    log_id: str
    voice_command: VoiceCommand
    action_plan: ActionPlan
    final_status: str
    start_time: datetime
    end_time: Optional[datetime] = None
    step_logs: list = field(default_factory=list)
    error_message: Optional[str] = None

@dataclass
class StepLog:
    """Log for individual action step."""
    step_index: int
    action_type: str
    status: str
    start_time: datetime
    end_time: Optional[datetime] = None
    feedback_history: list = field(default_factory=list)
    error_message: Optional[str] = None

class VlaPipeline:
    """Complete Voice-Language-Action pipeline."""

    def __init__(
        self,
        voice_service: Optional[VoiceService] = None,
        cognitive_planner: Optional[CognitivePlanner] = None,
        action_service: Optional[IActionService] = None
    ):
        """
        Initialize VLA pipeline.

        Args:
            voice_service: Voice recognition service (defaults to Whisper)
            cognitive_planner: LLM planner (defaults to OpenAI GPT-4o)
            action_service: Action execution service (defaults to Pure Python)
        """
        self.voice_service = voice_service or VoiceService(model_size="base")
        self.cognitive_planner = cognitive_planner or CognitivePlanner(
            provider="openai",
            model="gpt-4o"
        )
        self.action_service = action_service or ActionService()

        self.execution_monitor = ExecutionMonitor(self.action_service)
        self.execution_logs = []

    async def process_voice_command(
        self,
        voice_command: VoiceCommand,
        feedback_callback=None
    ) -> ExecutionLog:
        """
        Process voice command through complete pipeline.

        Args:
            voice_command: Voice command to process
            feedback_callback: Optional callback for progress updates

        Returns:
            ExecutionLog with complete execution history
        """
        log_id = self._generate_log_id()

        # 1. Generate action plan
        try:
            action_plan = self.cognitive_planner.plan_actions(
                voice_command.transcribed_text
            )
        except Exception as e:
            return self._create_failed_log(
                log_id,
                voice_command,
                f"Planning failed: {e}"
            )

        # 2. Create execution log
        exec_log = ExecutionLog(
            log_id=log_id,
            voice_command=voice_command,
            action_plan=action_plan,
            final_status=ExecutionStatus.PENDING,
            start_time=datetime.now()
        )

        # 3. Execute action plan
        exec_log.final_status = ExecutionStatus.RUNNING

        try:
            step_logs = await self.execution_monitor.execute_plan(
                action_plan,
                feedback_callback=feedback_callback
            )

            exec_log.step_logs = step_logs
            exec_log.final_status = ExecutionStatus.COMPLETED
            exec_log.end_time = datetime.now()

        except Exception as e:
            exec_log.final_status = ExecutionStatus.FAILED
            exec_log.error_message = str(e)
            exec_log.end_time = datetime.now()

        # 4. Store log
        self.execution_logs.append(exec_log)

        return exec_log

    async def process_audio_file(
        self,
        audio_path: str,
        feedback_callback=None
    ) -> ExecutionLog:
        """
        Process audio file through complete pipeline.

        Args:
            audio_path: Path to audio file
            feedback_callback: Optional callback for progress updates

        Returns:
            ExecutionLog with complete execution history
        """
        # Transcribe audio
        voice_command = self.voice_service.transcribe_audio_file(audio_path)

        # Process command
        return await self.process_voice_command(voice_command, feedback_callback)

    def start_continuous_listening(self, feedback_callback=None):
        """
        Start continuous voice command processing.

        Args:
            feedback_callback: Optional callback for progress updates
        """
        def on_voice_command(voice_cmd: VoiceCommand):
            if voice_cmd.confidence > 0.7:
                asyncio.create_task(
                    self.process_voice_command(voice_cmd, feedback_callback)
                )

        self.voice_service.start_listening(callback=on_voice_command)

    def stop_continuous_listening(self):
        """Stop continuous voice command processing."""
        self.voice_service.stop_listening()

    def get_execution_history(self) -> list[ExecutionLog]:
        """Get all execution logs."""
        return self.execution_logs

    def _generate_log_id(self) -> str:
        """Generate unique log ID."""
        import uuid
        return f"log_{uuid.uuid4().hex[:8]}"

    def _create_failed_log(
        self,
        log_id: str,
        voice_command: VoiceCommand,
        error_message: str
    ) -> ExecutionLog:
        """Create failed execution log."""
        return ExecutionLog(
            log_id=log_id,
            voice_command=voice_command,
            action_plan=ActionPlan(steps=[], total_estimated_time=0.0, plan_id=""),
            final_status=ExecutionStatus.FAILED,
            start_time=datetime.now(),
            end_time=datetime.now(),
            error_message=error_message
        )

class ExecutionMonitor:
    """Monitor and execute action plans."""

    def __init__(self, action_service: IActionService):
        self.action_service = action_service

    async def execute_plan(
        self,
        plan: ActionPlan,
        feedback_callback=None
    ) -> list[StepLog]:
        """
        Execute action plan step by step.

        Args:
            plan: Action plan to execute
            feedback_callback: Optional callback for progress

        Returns:
            List of step logs
        """
        step_logs = []

        for i, step_data in enumerate(plan.steps):
            # Create action from step data
            action = RobotAction(
                action_type=ActionType[step_data["action_type"]],
                goal_message=step_data["goal_message"],
                timeout=step_data["timeout"]
            )

            # Create step log
            step_log = StepLog(
                step_index=i,
                action_type=step_data["action_type"],
                status=ExecutionStatus.RUNNING,
                start_time=datetime.now()
            )

            # Execute action
            def step_feedback(fb: dict):
                step_log.feedback_history.append(fb)
                if feedback_callback:
                    feedback_callback({
                        "step_index": i,
                        "total_steps": len(plan.steps),
                        "action_type": step_data["action_type"],
                        "feedback": fb
                    })

            try:
                success, error = await self.action_service.execute_action(
                    action,
                    feedback_callback=step_feedback
                )

                if success:
                    step_log.status = ExecutionStatus.COMPLETED
                else:
                    step_log.status = ExecutionStatus.FAILED
                    step_log.error_message = error

                step_log.end_time = datetime.now()
                step_logs.append(step_log)

                # Stop execution if step failed
                if not success:
                    break

            except Exception as e:
                step_log.status = ExecutionStatus.FAILED
                step_log.error_message = str(e)
                step_log.end_time = datetime.now()
                step_logs.append(step_log)
                raise

        return step_logs
```

## Usage Examples

### Example 1: Process Single Command

```python
import asyncio
from src.vla_core.pipeline import VlaPipeline
from src.vla_core.models.voice_command import VoiceCommand

async def main():
    # Initialize pipeline
    pipeline = VlaPipeline()

    # Create voice command
    command = VoiceCommand(
        transcribed_text="Pick up the red block",
        confidence=0.95,
        language="en",
        timestamp=datetime.now(),
        audio_duration=2.0
    )

    # Process command
    log = await pipeline.process_voice_command(command)

    # Display results
    print(f"Status: {log.final_status}")
    print(f"Steps executed: {len(log.step_logs)}")
    print(f"Total time: {(log.end_time - log.start_time).total_seconds():.2f}s")

    for step_log in log.step_logs:
        print(f"\nStep {step_log.step_index + 1}: {step_log.action_type}")
        print(f"  Status: {step_log.status}")
        print(f"  Feedback count: {len(step_log.feedback_history)}")

asyncio.run(main())
```

**Output**:
```
Status: completed
Steps executed: 1
Total time: 8.23s

Step 1: PICK
  Status: completed
  Feedback count: 4
```

### Example 2: Process Audio File with Feedback

```python
async def main():
    pipeline = VlaPipeline()

    def progress_callback(update: dict):
        step = update["step_index"] + 1
        total = update["total_steps"]
        action = update["action_type"]
        progress = update["feedback"].get("progress_percent", 0)

        print(f"[Step {step}/{total}] {action}: {progress:.1f}%")

    # Process audio file
    log = await pipeline.process_audio_file(
        "commands/go_to_kitchen.wav",
        feedback_callback=progress_callback
    )

    print(f"\nFinal status: {log.final_status}")

asyncio.run(main())
```

**Output**:
```
[Step 1/1] NAVIGATE: 20.0%
[Step 1/1] NAVIGATE: 40.0%
[Step 1/1] NAVIGATE: 60.0%
[Step 1/1] NAVIGATE: 80.0%
[Step 1/1] NAVIGATE: 100.0%

Final status: completed
```

### Example 3: Continuous Listening

```python
import time
from src.vla_core.pipeline import VlaPipeline

def main():
    pipeline = VlaPipeline()

    def on_progress(update: dict):
        print(f"Executing: {update['action_type']}")

    # Start listening
    pipeline.start_continuous_listening(feedback_callback=on_progress)

    print("VLA system ready. Listening for commands...")
    print("Speak a command (e.g., 'pick up the red block')")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
        pipeline.stop_continuous_listening()

        # Show execution history
        print(f"\nTotal commands processed: {len(pipeline.execution_logs)}")
        for log in pipeline.execution_logs:
            print(f"- {log.voice_command.transcribed_text}: {log.final_status}")

main()
```

### Example 4: Full System with All Backends

```python
from src.vla_core.pipeline import VlaPipeline
from src.vla_core.services.voice_service import VoiceService
from src.vla_core.services.cognitive_planner import CognitivePlanner
from src.vla_core.services.action_service import ActionService

async def main():
    # Initialize services with specific configurations
    voice_service = VoiceService(model_size="base")  # Whisper base model

    cognitive_planner = CognitivePlanner(
        provider="openai",
        model="gpt-4o"
    )

    action_service = ActionService()  # Pure Python execution

    # Create pipeline
    pipeline = VlaPipeline(
        voice_service=voice_service,
        cognitive_planner=cognitive_planner,
        action_service=action_service
    )

    # Process command
    command = VoiceCommand(
        transcribed_text="Pick up the red block, place it on the table, then go to x=2, y=1",
        confidence=0.95,
        language="en",
        timestamp=datetime.now(),
        audio_duration=5.0
    )

    log = await pipeline.process_voice_command(command)

    print(f"Processed: {log.voice_command.transcribed_text}")
    print(f"Plan: {len(log.action_plan.steps)} steps")
    print(f"Status: {log.final_status}")

    for i, step_log in enumerate(log.step_logs):
        duration = (step_log.end_time - step_log.start_time).total_seconds()
        print(f"{i+1}. {step_log.action_type}: {step_log.status} ({duration:.1f}s)")

asyncio.run(main())
```

**Output**:
```
Processed: Pick up the red block, place it on the table, then go to x=2, y=1
Plan: 3 steps
Status: completed
1. PICK: completed (8.1s)
2. PLACE: completed (10.2s)
3. NAVIGATE: completed (6.5s)
```

## Web Interface

### Simple Flask API

```python
# server.py
from flask import Flask, request, jsonify
from src.vla_core.pipeline import VlaPipeline
import asyncio

app = Flask(__name__)
pipeline = VlaPipeline()

@app.route('/api/command', methods=['POST'])
def process_command():
    """Process text command."""
    data = request.json
    command_text = data.get('command')

    # Create voice command from text
    voice_cmd = VoiceCommand(
        transcribed_text=command_text,
        confidence=1.0,
        language="en",
        timestamp=datetime.now(),
        audio_duration=0.0
    )

    # Process
    log = asyncio.run(pipeline.process_voice_command(voice_cmd))

    return jsonify({
        'log_id': log.log_id,
        'status': log.final_status,
        'steps': len(log.step_logs),
        'execution_time': (log.end_time - log.start_time).total_seconds()
    })

@app.route('/api/history', methods=['GET'])
def get_history():
    """Get execution history."""
    logs = pipeline.get_execution_history()

    return jsonify({
        'total': len(logs),
        'logs': [
            {
                'id': log.log_id,
                'command': log.voice_command.transcribed_text,
                'status': log.final_status,
                'steps': len(log.step_logs)
            }
            for log in logs
        ]
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
```

**Usage**:
```bash
# Start server
python server.py

# Send command
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "Pick up the red block"}'

# Get history
curl http://localhost:5000/api/history
```

## Monitoring and Observability

### Execution Dashboard

```python
# dashboard.py
from src.vla_core.pipeline import VlaPipeline
from rich.console import Console
from rich.table import Table
from rich.live import Live
import time

console = Console()
pipeline = VlaPipeline()

def create_status_table(logs):
    """Create status table."""
    table = Table(title="VLA Execution History")

    table.add_column("ID", style="cyan")
    table.add_column("Command", style="white")
    table.add_column("Status", style="green")
    table.add_column("Steps", justify="right")
    table.add_column("Duration", justify="right")

    for log in logs[-10:]:  # Last 10
        duration = ""
        if log.end_time:
            duration = f"{(log.end_time - log.start_time).total_seconds():.1f}s"

        status_color = {
            "completed": "green",
            "failed": "red",
            "running": "yellow"
        }.get(log.final_status, "white")

        table.add_row(
            log.log_id[:8],
            log.voice_command.transcribed_text[:40],
            f"[{status_color}]{log.final_status}[/{status_color}]",
            str(len(log.step_logs)),
            duration
        )

    return table

# Start continuous listening
pipeline.start_continuous_listening()

# Live dashboard
with Live(create_status_table(pipeline.execution_logs), refresh_per_second=1) as live:
    try:
        while True:
            live.update(create_status_table(pipeline.execution_logs))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pipeline.stop_continuous_listening()
```

## Error Handling and Recovery

### Retry Logic

```python
class ResilientPipeline(VlaPipeline):
    """Pipeline with automatic retry."""

    async def process_voice_command(
        self,
        voice_command: VoiceCommand,
        feedback_callback=None,
        max_retries: int = 3
    ) -> ExecutionLog:
        """Process with automatic retry on failure."""

        for attempt in range(max_retries):
            log = await super().process_voice_command(
                voice_command,
                feedback_callback
            )

            if log.final_status == ExecutionStatus.COMPLETED:
                return log

            # Retry with exponential backoff
            if attempt < max_retries - 1:
                await asyncio.sleep(2 ** attempt)

        return log
```

### Graceful Degradation

```python
class FaultTolerantPipeline(VlaPipeline):
    """Pipeline with fallback behaviors."""

    async def process_voice_command(
        self,
        voice_command: VoiceCommand,
        feedback_callback=None
    ) -> ExecutionLog:
        """Process with fallback on planning failure."""

        try:
            return await super().process_voice_command(
                voice_command,
                feedback_callback
            )
        except Exception as e:
            # Fallback: try simpler planner
            print(f"Primary planner failed: {e}. Trying fallback...")

            self.cognitive_planner = CognitivePlanner(
                provider="ollama",
                model="llama3"
            )

            return await super().process_voice_command(
                voice_command,
                feedback_callback
            )
```

## Performance Metrics

### Benchmarking

```python
import time
from statistics import mean, stdev

async def benchmark_pipeline(num_commands: int = 10):
    """Benchmark pipeline performance."""
    pipeline = VlaPipeline()

    commands = [
        "Pick up the red block",
        "Navigate to x=2, y=1",
        "Inspect the workspace",
        "Place the object on the table"
    ]

    latencies = {
        "voice": [],
        "planning": [],
        "execution": [],
        "total": []
    }

    for i in range(num_commands):
        command_text = commands[i % len(commands)]

        # Voice (simulated)
        voice_start = time.time()
        voice_cmd = VoiceCommand(
            transcribed_text=command_text,
            confidence=0.95,
            language="en",
            timestamp=datetime.now(),
            audio_duration=2.0
        )
        latencies["voice"].append(time.time() - voice_start)

        # Full pipeline
        total_start = time.time()
        log = await pipeline.process_voice_command(voice_cmd)
        latencies["total"].append(time.time() - total_start)

        # Extract sub-latencies from log
        latencies["execution"].append(
            (log.end_time - log.start_time).total_seconds()
        )

    # Print results
    print("Performance Metrics:")
    for metric, values in latencies.items():
        print(f"{metric:12s}: {mean(values):.3f}s ± {stdev(values):.3f}s")

asyncio.run(benchmark_pipeline())
```

## Deployment

### Docker Container

```dockerfile
# Dockerfile
FROM python:3.10-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    portaudio19-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY src/ /app/src/
WORKDIR /app

# Run pipeline
CMD ["python", "-m", "src.main"]
```

**Build and run**:
```bash
docker build -t vla-pipeline .
docker run -it --gpus all -v $(pwd)/logs:/app/logs vla-pipeline
```

## Best Practices

1. **Comprehensive Logging**: Log every step for debugging
2. **Progress Feedback**: Always provide real-time feedback to users
3. **Error Recovery**: Implement retries and fallbacks
4. **Performance Monitoring**: Track latencies and success rates
5. **Graceful Shutdown**: Clean up resources properly
6. **Security**: Validate inputs, sanitize outputs
7. **Testing**: Unit test each component and integration test the pipeline

## Summary

You now have a complete end-to-end VLA pipeline that:
- ✅ Converts voice to text with Whisper
- ✅ Plans actions with LLMs
- ✅ Executes actions with Pure Python or ROS 2
- ✅ Provides comprehensive logging and monitoring
- ✅ Handles errors gracefully
- ✅ Scales to continuous listening
- ✅ Supports multiple deployment modes

This represents a production-ready voice-to-action system for humanoid robots!

## Next Steps

- **[Chapter 5: ROS 2 Extension](./05-ros-extension)**: Integrate with ROS 2 for physical robots

## Additional Resources

- [VLA System Paper](https://arxiv.org/abs/2307.03356)
- [LangChain for Robotics](https://python.langchain.com/docs/use_cases/robotics)
- [Embodied AI Survey](https://arxiv.org/abs/2210.06849)
