# Quickstart Guide: VLA Integration

**Date**: 2025-12-06
**Feature**: Vision-Language-Action (VLA) Integration
**Target Audience**: Developers setting up the voice-to-action pipeline

## Prerequisites

- **OS**: Ubuntu 22.04 LTS (or compatible Linux distribution)
- **Python**: 3.11 or later
- **ROS 2**: Humble Hawksbill
- **Hardware**:
  - GPU recommended (NVIDIA with CUDA for Whisper acceleration)
  - Microphone with audio input
  - Minimum 8GB RAM

## Installation Steps

### 1. Install ROS 2 Humble

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2 setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo Ignition (Fortress)

```bash
# Add Gazebo repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install Gazebo Fortress
sudo apt update
sudo apt install -y gz-fortress

# Install ROS 2 - Gazebo bridge
sudo apt install -y ros-humble-ros-gz
```

### 3. Set Up Python Environment

```bash
# Clone the repository
cd ~/
git clone <repository-url> ai-textbookfirst
cd ai-textbookfirst

# Create Python virtual environment
python3.11 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Install PyTorch (for Whisper GPU support)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install OpenAI Whisper
pip install openai-whisper

# Install LangGraph and LangChain
pip install langgraph langchain langchain-openai

# Install ROS 2 Python client
pip install rclpy
```

### 4. Configure ROS 2 Workspace

```bash
# Create ROS 2 workspace
mkdir -p ~/vla_ws/src
cd ~/vla_ws/src

# Link VLA package (symlink to avoid copying)
ln -s ~/ai-textbookfirst/src/vla_core .
ln -s ~/ai-textbookfirst/src/simulation .

# Build custom action interfaces
cd ~/vla_ws
colcon build --packages-select vla_interfaces

# Source workspace
echo "source ~/vla_ws/install/setup.bash" >> ~/.bashrc
source ~/vla_ws/install/setup.bash
```

### 5. Download Whisper Model

```bash
# Download Whisper medium model (runs on first use, but pre-cache recommended)
python -c "import whisper; whisper.load_model('medium')"
```

### 6. Set Environment Variables

Create `.env` file in project root:

```bash
cat > ~/ai-textbookfirst/.env << EOF
# OpenAI API key for LLM cognitive planning
OPENAI_API_KEY=your_api_key_here

# Whisper model size (tiny, base, small, medium, large)
WHISPER_MODEL=medium

# Audio settings
AUDIO_SAMPLE_RATE=16000
AUDIO_DEVICE_INDEX=0  # Use default microphone

# ROS 2 settings
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=1

# Logging
LOG_LEVEL=INFO
LOG_DIR=./logs
EOF
```

**Important**: Replace `your_api_key_here` with your actual OpenAI API key.

## Running the System

### 1. Launch Simulation Environment

```bash
# Terminal 1: Start Gazebo with humanoid robot
cd ~/vla_ws
source install/setup.bash
ros2 launch simulation humanoid_sim.launch.py
```

You should see Gazebo window with a humanoid robot in a simple room environment.

### 2. Start VLA Pipeline

```bash
# Terminal 2: Start the voice-to-action pipeline
cd ~/ai-textbookfirst
source venv/bin/activate
python -m vla_core.pipeline.vla_pipeline
```

Expected output:
```
[INFO] VLA Pipeline initialized
[INFO] Whisper model loaded: medium
[INFO] ROS 2 action clients connected
[INFO] Listening for voice commands... (speak now)
```

### 3. Test Voice Commands

Speak into your microphone. Try these commands:

**Simple Navigation**:
- "Move forward 2 meters"
- "Turn left 90 degrees"
- "Go to the table"

**Object Manipulation**:
- "Pick up the red block"
- "Place the cup on the shelf"
- "Grab the blue object"

**Multi-Step Tasks**:
- "Clean the room"
- "Set the table for dinner"

**Interruptions**:
- "Stop" (halt current action)
- "Pause" (pause execution)
- "Continue" (resume after pause)

### 4. Monitor Execution Logs

```bash
# Terminal 3: Watch execution logs in real-time
tail -f ~/ai-textbookfirst/logs/executions/$(date +%Y-%m-%d)/latest.json
```

## Verification Checklist

After running your first command, verify:

- [ ] Whisper transcription appears in console
- [ ] LLM generates an action plan (visible in logs)
- [ ] Plan passes safety validation
- [ ] Robot executes action in Gazebo simulation
- [ ] Execution log is created in `logs/executions/`

## Troubleshooting

### Microphone Not Detected
```bash
# List audio devices
python -c "import pyaudio; p = pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)[\"name\"]}') for i in range(p.get_device_count())]"

# Set correct device in .env file
AUDIO_DEVICE_INDEX=<your_device_number>
```

### Whisper GPU Not Working
```bash
# Verify CUDA installation
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"

# If False, reinstall PyTorch with correct CUDA version
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### ROS Action Servers Not Found
```bash
# List available action servers
ros2 action list

# If empty, check if simulation launched correctly
ros2 node list
```

### LLM API Errors
```bash
# Test OpenAI API key
python -c "from openai import OpenAI; client = OpenAI(); print(client.models.list())"

# If error, verify .env file has correct OPENAI_API_KEY
```

## Next Steps

1. **Run Tests**: `pytest tests/integration/test_pipeline_e2e.py`
2. **Explore Code**: Start with `src/vla_core/pipeline/vla_pipeline.py`
3. **Add Custom Commands**: Modify `src/vla_core/cognition/action_planner.py`
4. **Tune Performance**: Adjust latency budgets in config files

## Configuration Files

| File | Purpose |
|------|---------|
| `.env` | Environment variables and API keys |
| `src/vla_core/config.py` | System configuration (timeouts, thresholds) |
| `simulation/config/robot.yaml` | Robot capabilities and constraints |
| `simulation/worlds/household_world.sdf` | Simulation environment setup |

## Development Workflow

1. **Make Changes**: Edit code in `src/vla_core/`
2. **Run Tests**: `pytest tests/` (ensure all pass)
3. **Test Manually**: Launch simulation and pipeline, test voice commands
4. **Check Logs**: Review execution logs for errors or unexpected behavior
5. **Iterate**: Refine based on test results

## Performance Tuning

**Reduce Latency**:
- Use `whisper-small` instead of `medium` (faster, slightly less accurate)
- Set `OPENAI_MODEL=gpt-3.5-turbo` instead of `gpt-4` (faster LLM)
- Enable GPU for Whisper: verify CUDA is working

**Improve Accuracy**:
- Use `whisper-large` for better transcription
- Add more examples to LLM system prompt
- Increase safety validation strictness

## Resources

- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Ignition Tutorials](https://gazebosim.org/docs/fortress/tutorials)
- [Whisper Documentation](https://github.com/openai/whisper)
- [LangGraph Guide](https://langchain-ai.github.io/langgraph/)

## Support

For issues, see:
- Project README: `README.md`
- Architecture docs: `specs/001-vla-integration/plan.md`
- API contracts: `specs/001-vla-integration/contracts/`