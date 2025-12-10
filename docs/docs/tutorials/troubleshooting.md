---
sidebar_position: 4
---

# Troubleshooting Guide

Common issues and solutions for the VLA Integration system.

## Installation Issues

### PyAudio Installation Failed

**Symptoms**:
```
ERROR: Failed building wheel for pyaudio
error: command 'gcc' failed with exit status 1
```

**Solutions**:

**Linux**:
```bash
# Install PortAudio development files
sudo apt-get install portaudio19-dev python3-pyaudio

# Then retry pip install
pip install pyaudio
```

**macOS**:
```bash
# Install PortAudio via Homebrew
brew install portaudio

# Install PyAudio
pip install pyaudio
```

**Windows**:
```bash
# Download precompiled wheel from:
# https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio

# Install (adjust Python version)
pip install PyAudio‑0.2.11‑cp311‑cp311‑win_amd64.whl
```

### CUDA Not Detected

**Symptoms**:
```python
>>> import torch
>>> torch.cuda.is_available()
False
```

**Solutions**:

```bash
# 1. Check NVIDIA driver
nvidia-smi

# 2. Verify CUDA installation
nvcc --version

# 3. Reinstall PyTorch with correct CUDA version
pip uninstall torch torchvision torchaudio

# For CUDA 11.8
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# For CUDA 12.1
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 4. Test again
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}, Devices: {torch.cuda.device_count()}')"
```

### ROS 2 Build Errors

**Symptoms**:
```
CMake Error: Could not find package geometry_msgs
```

**Solutions**:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install missing dependencies
sudo apt install ros-humble-geometry-msgs ros-humble-action-msgs

# Clean and rebuild
cd vla_ws
rm -rf build/ install/ log/
colcon build --packages-select vla_interfaces
```

## Runtime Issues

### Microphone Not Found

**Symptoms**:
```
OSError: [Errno -9996] Invalid input device (no default output device)
```

**Solutions**:

```bash
# List available audio devices
python -m src.cli.vla_cli devices

# Output:
# Available Audio Devices:
#   0: Built-in Microphone
#   1: USB Audio Device
#   2: Logitech Webcam

# Set specific device in .env
AUDIO_DEVICE_INDEX=1
```

**Linux Permission Issues**:
```bash
# Add user to audio group
sudo usermod -a -G audio $USER

# Logout and login for changes to take effect
```

### OpenAI API Errors

#### Authentication Failed

**Symptoms**:
```
openai.AuthenticationError: Invalid API key
```

**Solutions**:

```bash
# 1. Verify API key is set in .env
cat .env | grep OPENAI_API_KEY

# 2. Test API key
python -c "from openai import OpenAI; client = OpenAI(); print(client.models.list())"

# 3. Check for trailing spaces or quotes
# Ensure .env has: OPENAI_API_KEY=sk-... (no quotes, no spaces)

# 4. Regenerate key at: https://platform.openai.com/api-keys
```

#### Rate Limit Exceeded

**Symptoms**:
```
openai.RateLimitError: Rate limit reached for gpt-4-turbo-preview
```

**Solutions**:

```bash
# 1. Wait and retry (rate limits reset after time)

# 2. Use exponential backoff (already implemented in LLMClient)

# 3. Upgrade OpenAI plan for higher limits

# 4. Use GPT-3.5 temporarily (in .env)
OPENAI_MODEL=gpt-3.5-turbo
```

#### Timeout Errors

**Symptoms**:
```
openai.Timeout: Request timed out
```

**Solutions**:

```bash
# Increase timeout in config.py
OPENAI_REQUEST_TIMEOUT=60.0  # seconds

# Check network connectivity
curl https://api.openai.com/v1/models -H "Authorization: Bearer $OPENAI_API_KEY"
```

### Whisper Performance Issues

#### Slow Transcription

**Symptoms**:
- Transcription takes >5 seconds per command
- CPU usage at 100%

**Solutions**:

**1. Use GPU Acceleration**:
```bash
# Verify CUDA availability
python -c "import torch; print(torch.cuda.is_available())"

# Set in .env
WHISPER_DEVICE=cuda
WHISPER_FP16=true
```

**2. Use Smaller Model**:
```bash
# In .env
WHISPER_MODEL=base  # Instead of medium or large

# Model comparison:
# tiny: ~32x realtime (less accurate)
# base: ~16x realtime (moderate accuracy)
# small: ~6x realtime (good accuracy)
# medium: ~2x realtime (very good accuracy)
# large: ~1x realtime (best accuracy)
```

**3. Reduce Audio Quality**:
```bash
# In .env
AUDIO_SAMPLE_RATE=8000  # Instead of 16000
```

#### Low Transcription Accuracy

**Symptoms**:
- Confidence scores consistently <0.7
- Incorrect words in transcription

**Solutions**:

**1. Upgrade Model**:
```bash
WHISPER_MODEL=medium  # or large
```

**2. Improve Audio Quality**:
- Use external USB microphone
- Reduce background noise
- Speak 15-30cm from microphone
- Speak clearly at moderate pace

**3. Adjust VAD Threshold**:
```bash
# Lower threshold for quieter speech
VAD_THRESHOLD=0.3
```

**4. Disable VAD (Process All Audio)**:
```bash
VAD_ENABLED=false
```

### Plan Validation Failures

#### Target Outside Workspace

**Symptoms**:
```
✗ Plan validation failed:
   - Step 2: Target pose outside workspace bounds
```

**Solutions**:

```bash
# Expand workspace in .env
WORKSPACE_BOUNDS='{"x_min": -2.0, "x_max": 4.0, "y_min": -3.0, "y_max": 3.0, "z_min": 0.0, "z_max": 2.5}'

# Or provide more specific location in command:
# Instead of: "Pick up the block"
# Say: "Pick up the block from the table"
```

#### Force Limit Exceeded

**Symptoms**:
```
✗ Step 3: Force limit exceeded (60.0N > 50.0N)
```

**Solutions**:

```bash
# Increase force limit (if safe for your robot)
MAX_FORCE_THRESHOLD=70.0

# Or modify command to use gentler grasp:
# "Gently pick up the fragile object"
```

#### Plan Too Complex

**Symptoms**:
```
✗ Plan exceeds maximum steps: 15 > 10
```

**Solutions**:

```bash
# 1. Increase step limit
MAX_PLAN_STEPS=20

# 2. Break command into smaller tasks:
# Instead of: "Sort all 20 blocks by color"
# Say: "Sort the red blocks" (then repeat for other colors)
```

### Execution Errors

#### Action Server Unavailable

**Symptoms**:
```
✗ Action server '/pick_object' not available
```

**Solutions**:

**In Mock Mode**:
```bash
# Ensure ROS_USE_MOCK=true in .env
ROS_USE_MOCK=true
```

**In Real Robot Mode**:
```bash
# Check ROS 2 action servers are running
ros2 action list

# Expected output:
#   /pick_object
#   /place_object
#   /navigate_to_point
#   /inspect_object

# Start missing servers
ros2 run vla_action_servers pick_object_server
```

#### Action Timeout

**Symptoms**:
```
✗ Step 2 failed: Action timeout after 30.0s
```

**Solutions**:

```bash
# Increase action timeout
ROS_ACTION_TIMEOUT=60.0

# Or in command, simplify task:
# Instead of: "Navigate across the entire building"
# Say: "Move forward 2 meters"
```

#### Execution Paused Unexpectedly

**Symptoms**:
- Execution stops mid-plan
- No error message

**Solutions**:

```python
# Check execution state
state = pipeline.execution_monitor.get_current_state()
print(f"Status: {state.status}")

# If status is PAUSED:
await pipeline.execution_monitor.resume_execution()
```

## Configuration Issues

### Environment Variables Not Loaded

**Symptoms**:
```
ValueError: OpenAI API key not set
```

**Solutions**:

```bash
# 1. Verify .env file exists
ls -la .env

# 2. Check file contents
cat .env | grep OPENAI_API_KEY

# 3. Ensure no syntax errors
# Correct: OPENAI_API_KEY=sk-...
# Wrong: OPENAI_API_KEY = "sk-..."  (no spaces, no quotes)

# 4. Explicitly load config
python -c "from src.vla_core.utils.config import load_config; config = load_config(env_file='.env'); print(config.openai_api_key[:10])"
```

### Invalid Configuration Values

**Symptoms**:
```
pydantic.ValidationError: 1 validation error for VLAConfig
  max_plan_steps
    ensure this value is greater than or equal to 1 (type=value_error)
```

**Solutions**:

```bash
# Check constraints in config.py
# Example: max_plan_steps must be between 1-50

# Fix .env
MAX_PLAN_STEPS=10  # Within valid range
```

## Performance Issues

### High Memory Usage

**Symptoms**:
- Python process using >8GB RAM
- System becomes unresponsive

**Causes & Solutions**:

**1. Large Whisper Model**:
```bash
# Use smaller model
WHISPER_MODEL=base  # ~1GB instead of medium (~5GB)
```

**2. Model Not Cached**:
```python
# Pre-load models once
python -c "import whisper; whisper.load_model('medium')"
```

**3. Memory Leak in Loop Mode**:
```bash
# Restart pipeline periodically
# Or use single-command mode instead of --loop
```

### High CPU Usage

**Symptoms**:
- CPU at 100% during idle
- Laptop overheating

**Solutions**:

```bash
# 1. Ensure GPU is used for Whisper
WHISPER_DEVICE=cuda

# 2. Reduce audio processing
VAD_ENABLED=true  # Skip processing silence

# 3. Increase sleep intervals in loops
AUDIO_CAPTURE_SLEEP=0.1  # seconds
```

## Logging Issues

### No Execution Logs Created

**Symptoms**:
- `logs/executions/` directory is empty

**Solutions**:

```bash
# 1. Verify logging is enabled
LOG_EXECUTION_TO_FILE=true

# 2. Check permissions
mkdir -p logs/executions
chmod 755 logs/executions

# 3. Check log level
LOG_LEVEL=INFO  # or DEBUG
```

### Log Files Too Large

**Symptoms**:
- Individual log files >100MB

**Solutions**:

```bash
# 1. Reduce log level
LOG_LEVEL=INFO  # Instead of DEBUG

# 2. Enable log rotation
LOG_ROTATION_SIZE_MB=10
LOG_RETENTION_DAYS=7

# 3. Clean old logs
find logs/executions -type f -mtime +7 -delete
```

## Network Issues

### Connection Timeout

**Symptoms**:
```
requests.exceptions.ConnectTimeout: HTTPSConnectionPool(host='api.openai.com', port=443)
```

**Solutions**:

```bash
# 1. Check internet connection
ping api.openai.com

# 2. Check proxy settings
echo $HTTP_PROXY
echo $HTTPS_PROXY

# 3. Increase timeout
OPENAI_REQUEST_TIMEOUT=120.0

# 4. Use different DNS
# Edit /etc/resolv.conf:
# nameserver 8.8.8.8
```

### SSL Certificate Errors

**Symptoms**:
```
ssl.SSLCertVerificationError: [SSL: CERTIFICATE_VERIFY_FAILED]
```

**Solutions**:

```bash
# 1. Update CA certificates (Linux)
sudo apt-get install --reinstall ca-certificates

# 2. Update certifi package
pip install --upgrade certifi

# 3. Temporarily disable verification (NOT recommended for production)
# In llm_client.py:
# openai.verify_ssl_certs = False
```

## Debugging Tips

### Enable Debug Logging

```bash
# In .env
LOG_LEVEL=DEBUG

# Or command-line
python -m src.cli.vla_cli --log-level DEBUG run
```

### Inspect Pipeline State

```python
from src.vla_core.pipeline.vla_pipeline import VlaPipeline

pipeline = VlaPipeline()
state_manager = pipeline.state_manager

# Check current execution
current_state = state_manager.get_current_execution_state()
print(f"Status: {current_state.status}")
print(f"Progress: {current_state.progress_percent}%")

# Inspect voice commands
for cmd_id, cmd in state_manager._voice_commands.items():
    print(f"{cmd_id}: {cmd.transcribed_text} ({cmd.confidence})")
```

### Test Individual Components

```python
# Test Whisper
from src.vla_core.voice.transcription import WhisperTranscriber
transcriber = WhisperTranscriber(model_name="base")
text, conf = await transcriber.transcribe(audio_data)

# Test Intent Parser
from src.vla_core.cognition.intent_parser import IntentParser
parser = IntentParser(config)
intent = await parser.parse(voice_command)

# Test ROS Interface
from src.vla_core.execution.ros_interface import RosInterface
ros = RosInterface(config)
success, error = await ros.execute_action(action)
```

### Examine Execution Logs

```bash
# View latest log
cat logs/executions/$(date +%Y-%m-%d)/*.json | jq .

# Filter for errors
jq 'select(.final_status == "failed")' logs/executions/*/*.json

# Show execution timeline
jq '.execution_trace[] | "\(.timestamp) [\(.event_type)] \(.message)"' logs/executions/2025-12-08/*.json
```

## Getting Help

### Check Logs

```bash
# System logs
cat logs/vla_pipeline.log

# Execution logs
cat logs/executions/YYYY-MM-DD/*.json

# ROS logs (if using real robot)
ros2 log list
ros2 log info /vla_pipeline_node
```

### Collect Diagnostic Info

```bash
# Run diagnostics script
python -m src.cli.vla_cli validate --verbose

# Collect system info
uname -a
python --version
pip list | grep -E "(whisper|openai|torch|rclpy)"
nvidia-smi  # If using GPU
```

### Report Issues

When reporting issues, include:

1. **Error message** (full stack trace)
2. **Command executed** (voice command or CLI command)
3. **Configuration** (.env file contents, redact API keys)
4. **System info** (OS, Python version, GPU)
5. **Log files** (recent execution logs)

## Next Steps

- [Architecture Overview](../architecture/overview)
- [API Reference](../api/overview)
- [Developer Guide](../developer-guide)