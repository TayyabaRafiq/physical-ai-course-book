# VLA Integration - Voice-Language-Action for Robot Control

A multi-modal voice-to-action pipeline for controlling humanoid robots through natural language commands. Integrates speech recognition (Whisper), cognitive planning (LLM-based), and pure Python execution to translate voice commands into executable robot actions.

## 🎯 Features

- **Voice Interface**: Local Whisper model for speech-to-text with GPU acceleration
- **Cognitive Planning**: LLM-based intent parsing and multi-step action planning
- **Safety Validation**: Pre-execution checks for joint limits, workspace bounds, and collisions
- **Pure Python Execution**: Zero-dependency action service (no ROS 2 required for MVP)
- **Execution Monitoring**: Real-time progress tracking with pause/resume/cancel support
- **Complete Traceability**: Structured execution logs with full audit trail
- **Interactive CLI**: Easy-to-use command-line interface with rich progress display
- **Platform Independent**: Works on Windows, macOS, and Linux

## ⭐ Why Pure Python?

The MVP uses a **Pure Python ActionService** instead of ROS 2, which means:
- ✅ **No external dependencies** - Just Python packages
- ✅ **Cross-platform** - Works on Windows, macOS, Linux
- ✅ **Easy setup** - No ROS 2 installation required
- ✅ **Fast iteration** - Test and develop without robot hardware
- ✅ **Extensible** - Add ROS 2 backend later for real robots

## 📋 System Requirements

- **OS**: Windows 10+, macOS 11+, or Linux (Ubuntu 20.04+)
- **Python**: 3.11 or later
- **Hardware**:
  - GPU optional (NVIDIA with CUDA for Whisper acceleration)
  - Microphone for voice input
  - Minimum 8GB RAM

## 🚀 Quick Start

### 1. Clone Repository

```bash
git clone <repository-url> ai-textbookfirst
cd ai-textbookfirst
```

### 2. Create Virtual Environment

```bash
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Install PyTorch with CUDA support (for GPU acceleration)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install development tools
pip install -e ".[dev]"
```

### 4. Configure Environment

```bash
# Copy environment template
cp .env.template .env

# Edit .env and add your OpenAI API key
nano .env  # or use your preferred editor
```

**Required Configuration**:
```env
OPENAI_API_KEY=your_api_key_here  # Get from https://platform.openai.com/api-keys
WHISPER_MODEL=medium              # Options: tiny, base, small, medium, large
```

### 5. List Audio Devices

```bash
python -m src.cli.vla_cli devices
```

Select your microphone and set `AUDIO_DEVICE_INDEX` in `.env`.

### 6. Validate Setup

```bash
python -m src.cli.vla_cli validate --check-api
```

### 7. Run the System

```bash
# Single command mode
python -m src.cli.vla_cli run

# Continuous mode (loop)
python -m src.cli.vla_cli run --loop --timeout 10
```

## 🎤 Usage Examples

### Voice Commands

```
"Pick up the red block"
"Move forward 2 meters"
"Place the cup on the table"
"Stop"
```

### CLI Commands

```bash
# Run pipeline
vla-cli run --timeout 10

# List audio devices
vla-cli devices

# Validate configuration
vla-cli validate --check-api

# Show version
vla-cli version
```

## 🏗️ Architecture

### Three-Layer Design

```
┌─────────────────────────────────────────┐
│  Voice Interface Layer                  │
│  ┌──────────┐    ┌─────────────┐       │
│  │ PyAudio  │───▶│   Whisper   │       │
│  │ Capture  │    │ Transcriber │       │
│  └──────────┘    └──────┬──────┘       │
└────────────────────────┼────────────────┘
                         │ text
                         ▼
┌─────────────────────────────────────────┐
│  Cognitive Planning Layer (LangGraph)   │
│  ┌──────────┐    ┌──────────┐          │
│  │  Intent  │───▶│  Action  │          │
│  │  Parser  │    │ Planner  │          │
│  └──────────┘    └─────┬────┘          │
│       │  (GPT-4)       │                │
│       │                ▼                │
│       │         ┌──────────────┐        │
│       └────────▶│  Validator   │        │
│                 └──────┬───────┘        │
└────────────────────────┼────────────────┘
                         │ ActionPlan
                         ▼
┌─────────────────────────────────────────┐
│  Execution Layer (Pure Python)          │
│  ┌──────────────┐    ┌──────────────┐  │
│  │  Execution   │───▶│   Action     │  │
│  │  Monitor     │    │   Service    │  │
│  └──────────────┘    └──────┬───────┘  │
│                            │            │
│       Simulated Progress & Feedback    │
└────────────────────────────┼────────────┘
                             │ ExecutionLog
                             ▼
                       [JSON Logs]

Optional Future Extension:
┌─────────────────────────────────────────┐
│  ROS 2 Backend (Optional)               │
│  ┌──────────────┐    ┌──────────────┐  │
│  │     ROS      │───▶│   Gazebo     │  │
│  │ Action Client│    │  Simulation  │  │
│  └──────────────┘    └──────────────┘  │
└─────────────────────────────────────────┘
```

### Pipeline Flow

1. **Voice Capture**: PyAudio captures microphone input
2. **Transcription**: Whisper converts speech to text
3. **Intent Parsing**: LLM extracts structured intent (action type, objects, parameters)
4. **Action Planning**: LLM generates multi-step action plan
5. **Validation**: Safety checks (joint limits, workspace, collisions)
6. **Execution**: Pure Python ActionService simulates execution with progress updates
7. **Monitoring**: Real-time progress tracking and logging

## 📁 Project Structure

```
ai-textbookfirst/
├── src/
│   ├── vla_core/              # Core VLA system
│   │   ├── models/            # Pydantic data models
│   │   ├── voice/             # Audio capture + Whisper
│   │   ├── cognition/         # LLM-based planning
│   │   │   └── prompts/       # LLM system prompts
│   │   ├── execution/         # Pure Python action service
│   │   ├── pipeline/          # Main orchestrator
│   │   ├── contracts/         # Interface definitions
│   │   └── utils/             # Config, logging, errors
│   ├── simulation/            # Gazebo simulation
│   │   ├── worlds/            # Simulation environments
│   │   ├── robot_description/ # Robot URDF/SDF
│   │   ├── launch/            # ROS 2 launch files
│   │   └── action_servers/    # ROS action servers
│   └── cli/                   # Command-line interface
├── vla_ws/                    # ROS 2 workspace
│   └── src/
│       └── vla_interfaces/    # Custom ROS 2 actions
├── tests/                     # Test suite
├── logs/                      # Execution logs
├── specs/                     # Feature specifications
├── requirements.txt           # Python dependencies
├── pyproject.toml            # Project configuration
└── README.md                 # This file
```

## 🔧 Configuration

All configuration is in `.env` file. Key settings:

### OpenAI API
```env
OPENAI_API_KEY=sk-...         # Required
OPENAI_MODEL=gpt-4-turbo      # LLM model
LLM_TIMEOUT=10.0              # Request timeout
```

### Whisper
```env
WHISPER_MODEL=medium          # tiny|base|small|medium|large
WHISPER_USE_GPU=true          # Enable GPU acceleration
WHISPER_CONFIDENCE_THRESHOLD=0.7  # Min confidence
```

### Audio
```env
AUDIO_SAMPLE_RATE=16000       # Sample rate (Hz)
AUDIO_DEVICE_INDEX=-1         # Device index (-1=default)
VAD_THRESHOLD=0.5             # Voice activity threshold
```

### Safety
```env
MAX_PLAN_STEPS=10             # Maximum plan complexity
MAX_FORCE_THRESHOLD=50.0      # Max gripper force (N)
MAX_VELOCITY=0.5              # Max velocity (m/s)
ENABLE_COLLISION_CHECKING=true
```

See `.env.template` for all options.

## 📊 Data Models

### Core Entities

- **VoiceCommand**: Captured audio with transcription
- **ParsedIntent**: Structured command interpretation
- **ActionPlan**: Sequence of robot actions
- **RobotAction**: Single executable step
- **ExecutionState**: Real-time execution status
- **ExecutionLog**: Persistent audit trail

### Enums

- **ActionType**: NAVIGATE, PICK, PLACE, INSPECT, MANIPULATE, WAIT, STOP
- **ExecutionStatus**: IDLE, RUNNING, PAUSED, COMPLETED, FAILED, CANCELLED

## 🧪 Development

### Install Development Dependencies

```bash
pip install -e ".[dev]"
```

### Run Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test
pytest tests/unit/test_voice/test_audio_capture.py
```

### Code Quality

```bash
# Format code
black src/ tests/
isort src/ tests/

# Type checking
mypy src/

# Linting
flake8 src/ tests/
```

## 🔐 Security Notes

- **API Keys**: Never commit `.env` file - use `.env.template`
- **Audio Data**: Stored temporarily, not persisted by default
- **Execution Logs**: May contain sensitive command data - configure retention

## 🐛 Troubleshooting

### Whisper Model Not Loading

```bash
# Manually download model
python -c "import whisper; whisper.load_model('medium')"
```

### OpenAI API Errors

```bash
# Test API key
python -c "from openai import OpenAI; client = OpenAI(); print(client.models.list())"
```

### Microphone Not Detected

```bash
# List devices
python -m src.cli.vla_cli devices

# Set device index in .env
AUDIO_DEVICE_INDEX=<your_device_number>
```

### Import Errors

```bash
# Ensure virtual environment is activated
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows

# Reinstall in development mode
pip install -e .
```

## 📖 Documentation

- **Architecture**: `specs/001-vla-integration/plan.md`
- **Data Model**: `specs/001-vla-integration/data-model.md`
- **API Contracts**: `specs/001-vla-integration/contracts/`
- **Quickstart**: `specs/001-vla-integration/quickstart.md`

## 🎯 Roadmap

### MVP (Current) ✅
- [x] Voice interface with Whisper
- [x] LLM-based intent parsing
- [x] Multi-step action planning with validation
- [x] Pure Python execution layer
- [x] Enhanced CLI with plan visualization
- [x] Execution logging and tracing

### Phase 2 (User Story 2) ✅
- [x] Multi-step task decomposition
- [x] Complex command planning
- [x] Gripper state validation
- [x] Plan visualization in CLI

### Phase 3 (User Story 3) - Next
- [ ] Real-time voice interruptions
- [ ] Pause/resume functionality
- [ ] Emergency stop handling
- [ ] State persistence

### Future Extensions (Optional)
- [ ] ROS 2 backend integration
- [ ] Gazebo simulation deployment
- [ ] Physical robot testing
- [ ] GPU optimization
- [ ] CI/CD pipeline

## 🤝 Contributing

This is a research prototype. For issues or questions, contact the development team.

## 📄 License

MIT License - See LICENSE file for details.

## 🙏 Acknowledgments

- **OpenAI Whisper**: Speech recognition model
- **OpenAI API**: LLM inference
- **LangChain/LangGraph**: Agentic framework
- **Click**: CLI framework
- **Pydantic**: Data validation
- **ROS 2 & Gazebo** (Optional): Future robot control integration

---

**Version**: 0.1.0 (MVP)
**Status**: Development
**Last Updated**: 2025-12-06