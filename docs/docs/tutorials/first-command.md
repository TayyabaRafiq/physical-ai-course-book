---
sidebar_position: 2
---

# First Voice Command

Learn how to execute your first voice command through the VLA pipeline.

## Prerequisites

- Completed [Installation Guide](installation)
- Microphone connected and tested
- OpenAI API key configured in `.env`
- Virtual environment activated

## Starting the Pipeline

### 1. Activate Virtual Environment

```bash
# Linux/macOS
source venv/bin/activate

# Windows
venv\Scripts\activate
```

### 2. Verify Configuration

```bash
python -m src.cli.vla_cli validate
```

### 3. Start Interactive Mode

```bash
python -m src.cli.vla_cli run
```

**Expected output**:
```
VLA Integration Pipeline v0.1.0
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🎤 Voice Interface: Whisper (medium, cuda)
🧠 Cognition: GPT-4 Turbo
🤖 Execution: ROS 2 (Mock Mode)

Ready to listen. Speak your command...
```

## Your First Command

### Simple PICK Command

**Speak clearly**: "Pick up the red block"

### What Happens

The system processes your command through 6 stages:

#### Stage 1: Voice Capture (1-2 seconds)

```
🎤 Listening... (timeout: 10s)
🎤 Audio captured: 2.3s
🎤 Transcribing with Whisper...
✓ Transcription: "Pick up the red block" (confidence: 0.95)
```

#### Stage 2: Intent Parsing (1-2 seconds)

```
🧠 Parsing intent with LLM...
✓ Intent parsed:
   Action: PICK
   Target: red block (block, red, confidence: 0.92)
   Parameters: {"grasp_type": "top", "approach_offset": {"z": 0.1}}
   Clarification: No
```

#### Stage 3: Action Planning (1-3 seconds)

```
🧠 Generating action plan...
✓ Plan generated (2 steps):
   1. NAVIGATE → /navigate_to_point (target: near table)
   2. PICK → /pick_object (object: red_block_1, grasp: top)
   Estimated duration: 18.0s
```

#### Stage 4: Plan Validation (<100ms)

```
✓ Validating plan...
   ✓ Step count: 2/10
   ✓ Force limits: OK
   ✓ Velocity limits: OK
   ✓ Workspace bounds: OK
   ✓ Topological order: OK
✓ Plan validated successfully
```

#### Stage 5: Execution (5-30 seconds)

```
🤖 Executing plan...
   [1/2] NAVIGATE → /navigate_to_point
         Progress: ████████░░ 80% (Approaching target)
   ✓ Navigation complete (8.2s)

   [2/2] PICK → /pick_object
         Progress: ████████░░ 80% (Closing gripper)
   ✓ Pick complete (5.1s)

✓ Plan execution complete (13.3s total)
```

#### Stage 6: Logging

```
📝 Execution log saved: logs/executions/2025-12-08/550e8400-e29b-41d4-a716-446655440000.json
```

## Understanding the Output

### Voice Command Object

```json
{
  "id": "660e8400-...",
  "transcribed_text": "Pick up the red block",
  "confidence": 0.95,
  "timestamp": "2025-12-08T10:30:00Z"
}
```

### Parsed Intent

```json
{
  "action_type": "PICK",
  "target_objects": [
    {
      "name": "red block",
      "type": "block",
      "color": "red",
      "confidence": 0.92
    }
  ],
  "parameters": {
    "grasp_type": "top",
    "approach_offset": {"z": 0.1}
  },
  "requires_clarification": false
}
```

### Action Plan

```json
{
  "steps": [
    {
      "action_type": "NAVIGATE",
      "ros_action_name": "/navigate_to_point",
      "goal_message": {"target_location": "near table"},
      "timeout": 10.0
    },
    {
      "action_type": "PICK",
      "ros_action_name": "/pick_object",
      "goal_message": {"object_id": "red_block_1", "grasp_type": "top"},
      "timeout": 8.0
    }
  ],
  "estimated_duration": 18.0
}
```

## Trying Different Commands

### PLACE Command

**Speak**: "Place the block on the shelf"

**Expected plan**:
1. NAVIGATE to shelf
2. PLACE object at shelf location

### NAVIGATE Command

**Speak**: "Move to the table"

**Expected plan**:
1. NAVIGATE to table position

### INSPECT Command

**Speak**: "Look at the blue cup"

**Expected plan**:
1. NAVIGATE near blue cup
2. INSPECT blue cup

## Handling Low Confidence

If Whisper confidence is below threshold (default 0.7):

```
⚠ Low confidence transcription: "Pick up the... block" (confidence: 0.62)
🎤 Please repeat your command (attempt 2/3)
```

**Tips for better recognition**:
- Speak clearly and at moderate pace
- Reduce background noise
- Position microphone 15-30cm from mouth
- Use simple, direct commands

## Mock Mode vs Real Robot

### Mock Mode (Default)

```bash
# In .env
ROS_USE_MOCK=true
```

- Simulates action execution with delays
- No physical robot required
- Useful for testing pipeline flow
- Always succeeds (no physics)

### Real Robot Mode

```bash
# In .env
ROS_USE_MOCK=false
```

- Requires ROS 2 action servers running
- Uses actual robot hardware
- Subject to real-world constraints
- May fail due to physics/perception

## Command Line Options

### Timeout

```bash
# Set voice command timeout (default: 10s)
python -m src.cli.vla_cli run --timeout 15.0
```

### Loop Mode

```bash
# Continuous listening (exit with Ctrl+C)
python -m src.cli.vla_cli run --loop
```

### Log Level

```bash
# Increase verbosity
python -m src.cli.vla_cli --log-level DEBUG run
```

### Config File

```bash
# Use custom config
python -m src.cli.vla_cli --config my-config.env run
```

## Viewing Execution Logs

Logs are saved to `logs/executions/YYYY-MM-DD/`:

```bash
# View latest log
cat logs/executions/$(date +%Y-%m-%d)/*.json | jq .

# Check final status
cat logs/executions/$(date +%Y-%m-%d)/*.json | jq '.final_status'

# View execution trace
cat logs/executions/$(date +%Y-%m-%d)/*.json | jq '.execution_trace[]'
```

**Example trace entry**:

```json
{
  "timestamp": "2025-12-08T10:30:05Z",
  "event_type": "ACTION_START",
  "step_index": 0,
  "message": "Starting NAVIGATE: /navigate_to_point"
}
```

## Common Issues

### "Microphone not found"

```bash
# List available devices
python -m src.cli.vla_cli devices

# Set specific device in .env
AUDIO_DEVICE_INDEX=1
```

### "OpenAI API error"

```bash
# Verify API key
python -c "from src.vla_core.utils.config import load_config; print(load_config().openai_api_key[:10])"

# Check API status: https://status.openai.com/
```

### "Whisper takes too long"

```bash
# Use smaller model (faster but less accurate)
WHISPER_MODEL=base

# Or ensure GPU is being used
python -c "import torch; print(torch.cuda.is_available())"
```

### "Plan validation failed"

Check error message for specific constraint violation:

```
✗ Plan validation failed:
   - Step 1: Target pose outside workspace bounds
   - Step 2: Force limit exceeded (60.0N > 50.0N)
```

Adjust workspace or safety limits in `.env` if needed.

## Next Steps

- [Multi-Step Task Tutorial](multi-step-task) - Learn complex task execution
- [Architecture Overview](../architecture/overview) - Understand system design
- [Troubleshooting Guide](troubleshooting) - Fix common problems

## Advanced Usage

### Programmatic API

Instead of CLI, use Python API directly:

```python
from src.vla_core.pipeline.vla_pipeline import VlaPipeline

async def main():
    pipeline = VlaPipeline()
    await pipeline.initialize()

    # Process single command
    log = await pipeline.process_voice_command(timeout=10.0)

    print(f"Status: {log.final_status}")
    print(f"Duration: {log.total_duration}s")

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
```

### Custom Callbacks

Monitor execution progress:

```python
def state_callback(state):
    print(f"Progress: {state.progress_percent:.1f}%")
    print(f"Current step: {state.current_step_index}")

log = await pipeline.process_voice_command(
    timeout=10.0,
    state_callback=state_callback
)
```