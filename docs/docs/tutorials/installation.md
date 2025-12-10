---
sidebar_position: 1
---

# Installation Guide

Complete setup instructions for the VLA Integration system.

## Prerequisites

### Hardware Requirements

- **CPU**: Modern multi-core processor (Intel i5/AMD Ryzen 5 or better)
- **RAM**: Minimum 8GB, recommended 16GB+
- **GPU**: Optional but recommended for Whisper inference (NVIDIA with CUDA support)
- **Storage**: 10GB free space for models and dependencies
- **Microphone**: USB or built-in microphone for voice input

### Software Requirements

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **Python**: 3.11 or later
- **Git**: For cloning the repository
- **ROS 2**: Humble Hawksbill (for full system integration)

## Installation Steps

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/vla-integration.git
cd vla-integration
```

### 2. Create Python Virtual Environment

```bash
# Create virtual environment
python3.11 -m venv venv

# Activate (Linux/macOS)
source venv/bin/activate

# Activate (Windows)
venv\Scripts\activate
```

### 3. Install Python Dependencies

```bash
# Upgrade pip
pip install --upgrade pip

# Install core dependencies
pip install -r requirements.txt
```

**Key Dependencies Installed**:
- `openai-whisper` - Speech recognition
- `langgraph` - Agentic planning
- `langchain-openai` - LLM integration
- `pydantic>=2.5.0` - Data validation
- `pyaudio` - Audio capture
- `rclpy` - ROS 2 Python client (optional)

### 4. Install PyTorch with CUDA (Optional but Recommended)

For GPU-accelerated Whisper inference:

```bash
# For CUDA 11.8
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# For CUDA 12.1
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# For CPU only
pip install torch torchvision torchaudio
```

Verify GPU availability:

```python
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### 5. Download Whisper Models

Whisper models are downloaded automatically on first use, but you can pre-download:

```python
python -c "import whisper; whisper.load_model('medium')"
```

**Model sizes**:
- `tiny` - 39M params (~75MB)
- `base` - 74M params (~140MB)
- `small` - 244M params (~460MB)
- `medium` - 769M params (~1.5GB) **← Recommended**
- `large` - 1550M params (~2.9GB)

### 6. Configure Environment Variables

Create `.env` file from template:

```bash
cp .env.template .env
```

Edit `.env` and set required values:

```bash
# OpenAI API Configuration
OPENAI_API_KEY=sk-your-api-key-here
OPENAI_MODEL=gpt-4-turbo-preview

# Whisper Configuration
WHISPER_MODEL=medium
WHISPER_DEVICE=cuda  # or 'cpu'
WHISPER_CONFIDENCE_THRESHOLD=0.7

# ROS 2 Configuration (for full system)
ROS_USE_MOCK=true  # Set to false for real robot
ROS_DOMAIN_ID=0

# Safety Limits
MAX_FORCE_THRESHOLD=50.0
MAX_VELOCITY_THRESHOLD=0.5
WORKSPACE_BOUNDS={"x_min": -1.0, "x_max": 3.0, "y_min": -2.0, "y_max": 2.0, "z_min": 0.0, "z_max": 2.0}
```

### 7. Test Installation

```bash
# Verify configuration
python -m src.cli.vla_cli --version

# Test audio capture
python -m src.cli.vla_cli devices

# Expected output:
# Available Audio Devices:
#   0: Built-in Microphone
#   1: USB Audio Device
```

## ROS 2 Setup (Optional)

For full robot integration, install ROS 2 Humble:

### Ubuntu 22.04

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build ROS 2 Action Interfaces

```bash
# Navigate to ROS workspace
cd vla_ws

# Build interfaces
colcon build --packages-select vla_interfaces

# Source workspace
source install/setup.bash
```

## Gazebo Simulation Setup (Optional)

For testing with simulated robot:

```bash
# Install Gazebo Ignition
sudo apt install ros-humble-ros-gz

# Test simulation
ros2 launch src/simulation/launch/humanoid_sim.launch.py
```

## Troubleshooting

### PyAudio Installation Fails (Linux)

```bash
# Install PortAudio development files
sudo apt-get install portaudio19-dev python3-pyaudio

# Then retry
pip install pyaudio
```

### PyAudio Installation Fails (Windows)

Download precompiled wheel:

```bash
# Download from https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio
pip install PyAudio‑0.2.11‑cp311‑cp311‑win_amd64.whl
```

### CUDA Not Detected

```bash
# Check NVIDIA driver
nvidia-smi

# Verify CUDA installation
nvcc --version

# Reinstall PyTorch with correct CUDA version
pip uninstall torch
pip install torch --index-url https://download.pytorch.org/whl/cu118
```

### OpenAI API Key Issues

```bash
# Verify key is set
python -c "from src.vla_core.utils.config import load_config; config = load_config(); print(f'API Key: {config.openai_api_key[:8]}...')"

# Test API connection
python -c "import openai; client = openai.OpenAI(); print(client.models.list())"
```

### Microphone Permission Denied (Linux)

```bash
# Add user to audio group
sudo usermod -a -G audio $USER

# Logout and login again
```

## Verification

Run the validation command to check all components:

```bash
python -m src.cli.vla_cli validate
```

**Expected output**:
```
✓ Python version: 3.11.5
✓ OpenAI API: Connected
✓ Whisper model: medium loaded
✓ Audio device: Available (device 0)
✓ Configuration: Valid
✓ ROS 2: Mock mode (or Connected if real ROS)

All systems operational!
```

## Next Steps

- [First Voice Command Tutorial](first-command)
- [Multi-Step Task Tutorial](multi-step-task)
- [Architecture Overview](../architecture/overview)

## Quick Start

Once installed, try your first command:

```bash
# Start the pipeline
python -m src.cli.vla_cli run

# Speak: "Pick up the red block"
# System will process and execute (or simulate in mock mode)
```

## Updating

To update to the latest version:

```bash
# Pull latest changes
git pull origin main

# Update dependencies
pip install -r requirements.txt --upgrade

# Rebuild ROS interfaces (if changed)
cd vla_ws && colcon build
```

## Uninstallation

```bash
# Remove virtual environment
deactivate
rm -rf venv/

# Remove Whisper models cache
rm -rf ~/.cache/whisper/

# Remove logs
rm -rf logs/
```