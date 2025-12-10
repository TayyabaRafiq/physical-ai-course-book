# Voice Interface with OpenAI Whisper

## Overview

The Voice Interface is the entry point to the VLA (Vision-Language-Action) pipeline. It uses OpenAI's Whisper model to convert spoken natural language commands into text that can be processed by the cognitive planning layer.

**Key Features**:
- High-accuracy speech recognition (>95% word accuracy)
- Multi-language support (99+ languages)
- Robust to accents and background noise
- Real-time and batch processing modes
- Local execution (no cloud API required)

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Audio Input                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │Microphone│  │Audio File│  │  Stream  │      │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘      │
└───────┼─────────────┼─────────────┼─────────────┘
        │             │             │
        └─────────────┴─────────────┘
                      ▼
┌─────────────────────────────────────────────────┐
│         Audio Preprocessing                     │
│  • Resampling to 16kHz                         │
│  • Noise reduction (optional)                  │
│  • Voice Activity Detection (VAD)              │
└──────────────────┬──────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────┐
│      Whisper Model (GPU-Accelerated)           │
│  • Encoder: Audio → Features                   │
│  • Decoder: Features → Tokens → Text           │
└──────────────────┬──────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────┐
│         VoiceCommand Output                     │
│  {                                              │
│    transcribed_text: "pick up the red block",  │
│    confidence: 0.95,                            │
│    language: "en",                              │
│    timestamp: ...                               │
│  }                                              │
└─────────────────────────────────────────────────┘
```

## Installation

### Install Whisper

```bash
# Install OpenAI Whisper
pip install openai-whisper

# For faster inference (optional)
pip install openai-whisper[speedup]

# For GPU support (recommended)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Install Audio Dependencies

```bash
# Linux
sudo apt install portaudio19-dev python3-pyaudio ffmpeg

# Install Python audio libraries
pip install pyaudio sounddevice soundfile numpy
```

### Verify Installation

```python
import whisper
import torch

# Check GPU availability
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None'}")

# Load model
model = whisper.load_model("base")
print(f"Model loaded: {model}")
```

## Model Selection

Whisper offers multiple model sizes with speed-accuracy tradeoffs:

| Model | Parameters | VRAM | Speed (GPU) | Word Error Rate |
|-------|------------|------|-------------|-----------------|
| **tiny** | 39M | ~1GB | ~10x realtime | ~5% |
| **base** | 74M | ~1GB | ~7x realtime | ~4% |
| **small** | 244M | ~2GB | ~4x realtime | ~3% |
| **medium** | 769M | ~5GB | ~2x realtime | ~2.5% |
| **large** | 1550M | ~10GB | ~1x realtime | ~2% |

**Recommendation**: Use `base` for real-time robotics applications.

## Voice Interface Implementation

### Core VoiceService

```python
# src/vla_core/services/voice_service.py
import whisper
import numpy as np
import sounddevice as sd
import queue
import threading
from dataclasses import dataclass
from datetime import datetime

@dataclass
class VoiceCommand:
    """Voice command data structure."""
    transcribed_text: str
    confidence: float
    language: str
    timestamp: datetime
    audio_duration: float

class VoiceService:
    """Voice recognition service using Whisper."""

    def __init__(self, model_size: str = "base"):
        """
        Initialize voice service.

        Args:
            model_size: Whisper model size (tiny, base, small, medium, large)
        """
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000
        self.audio_queue = queue.Queue()
        self.is_listening = False

    def transcribe_audio_file(self, audio_path: str) -> VoiceCommand:
        """
        Transcribe audio from file.

        Args:
            audio_path: Path to audio file (mp3, wav, etc.)

        Returns:
            VoiceCommand with transcription results
        """
        result = self.model.transcribe(
            audio_path,
            fp16=False,  # Use fp32 for better compatibility
            language="en"  # Or None for auto-detection
        )

        return VoiceCommand(
            transcribed_text=result["text"].strip(),
            confidence=self._calculate_confidence(result),
            language=result["language"],
            timestamp=datetime.now(),
            audio_duration=result.get("duration", 0.0)
        )

    def start_listening(self, callback):
        """
        Start continuous microphone listening.

        Args:
            callback: Function to call with VoiceCommand when speech detected
        """
        if self.is_listening:
            return

        self.is_listening = True
        self._listening_thread = threading.Thread(
            target=self._listen_loop,
            args=(callback,),
            daemon=True
        )
        self._listening_thread.start()

    def stop_listening(self):
        """Stop microphone listening."""
        self.is_listening = False
        if hasattr(self, '_listening_thread'):
            self._listening_thread.join(timeout=2.0)

    def _listen_loop(self, callback):
        """Main listening loop."""
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            callback=self._audio_callback
        ):
            while self.is_listening:
                # Process audio chunks
                if not self.audio_queue.empty():
                    audio_chunk = self._get_audio_chunk()
                    if self._is_speech(audio_chunk):
                        command = self._transcribe_chunk(audio_chunk)
                        callback(command)

    def _audio_callback(self, indata, frames, time, status):
        """Callback for audio stream."""
        if status:
            print(f"Audio status: {status}")
        self.audio_queue.put(indata.copy())

    def _get_audio_chunk(self, duration: float = 3.0) -> np.ndarray:
        """Get audio chunk of specified duration."""
        num_samples = int(duration * self.sample_rate)
        audio_data = []

        while len(audio_data) < num_samples:
            if not self.audio_queue.empty():
                audio_data.append(self.audio_queue.get())
            else:
                break

        if not audio_data:
            return np.array([])

        return np.concatenate(audio_data)[:num_samples]

    def _is_speech(self, audio: np.ndarray) -> bool:
        """
        Simple Voice Activity Detection (VAD).

        Args:
            audio: Audio samples

        Returns:
            True if speech detected
        """
        if len(audio) == 0:
            return False

        # Simple energy-based VAD
        energy = np.sqrt(np.mean(audio**2))
        threshold = 0.01  # Tune based on microphone

        return energy > threshold

    def _transcribe_chunk(self, audio: np.ndarray) -> VoiceCommand:
        """Transcribe audio chunk."""
        # Flatten and convert to float32
        audio_flat = audio.flatten().astype(np.float32)

        result = self.model.transcribe(
            audio_flat,
            fp16=False,
            language="en"
        )

        return VoiceCommand(
            transcribed_text=result["text"].strip(),
            confidence=self._calculate_confidence(result),
            language=result["language"],
            timestamp=datetime.now(),
            audio_duration=len(audio_flat) / self.sample_rate
        )

    def _calculate_confidence(self, result: dict) -> float:
        """
        Calculate confidence score from Whisper result.

        Args:
            result: Whisper transcription result

        Returns:
            Confidence score (0.0-1.0)
        """
        # Whisper doesn't provide direct confidence scores
        # Use average log probability as proxy
        segments = result.get("segments", [])
        if not segments:
            return 0.5

        avg_logprob = np.mean([seg.get("avg_logprob", -1.0) for seg in segments])

        # Convert log prob to confidence (heuristic)
        # Higher avg_logprob (closer to 0) = higher confidence
        confidence = np.clip(np.exp(avg_logprob), 0.0, 1.0)

        return confidence
```

## Usage Examples

### Example 1: Transcribe Audio File

```python
from src.vla_core.services.voice_service import VoiceService

# Initialize service
voice_service = VoiceService(model_size="base")

# Transcribe file
command = voice_service.transcribe_audio_file("command.wav")

print(f"Transcribed: {command.transcribed_text}")
print(f"Confidence: {command.confidence:.2f}")
print(f"Language: {command.language}")
```

### Example 2: Continuous Listening

```python
from src.vla_core.services.voice_service import VoiceService

def handle_command(command):
    print(f"[{command.timestamp}] {command.transcribed_text} (conf: {command.confidence:.2f})")

    # Process command
    if command.confidence > 0.7:
        # Send to cognitive planning
        pass

# Initialize and start listening
voice_service = VoiceService(model_size="base")
voice_service.start_listening(callback=handle_command)

print("Listening... Press Ctrl+C to stop")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    voice_service.stop_listening()
    print("Stopped listening")
```

### Example 3: Integration with VLA Pipeline

```python
from src.vla_core.pipeline import VlaPipeline
from src.vla_core.services.voice_service import VoiceService

async def voice_to_action():
    # Initialize services
    voice_service = VoiceService(model_size="base")
    pipeline = VlaPipeline()

    def on_voice_command(voice_cmd):
        # Create VoiceCommand for pipeline
        if voice_cmd.confidence > 0.7:
            asyncio.create_task(
                pipeline.process_voice_command(voice_cmd)
            )

    # Start listening
    voice_service.start_listening(callback=on_voice_command)

    print("Voice-to-action system ready!")
    await asyncio.Event().wait()  # Run forever

# Run
asyncio.run(voice_to_action())
```

## Advanced Features

### Multi-Language Support

```python
# Auto-detect language
result = model.transcribe(audio, language=None)
print(f"Detected language: {result['language']}")

# Force specific language
result = model.transcribe(audio, language="es")  # Spanish
```

### Timestamped Transcription

```python
result = model.transcribe(audio, word_timestamps=True)

for segment in result["segments"]:
    print(f"[{segment['start']:.2f}s - {segment['end']:.2f}s] {segment['text']}")
    for word in segment.get("words", []):
        print(f"  {word['start']:.2f}s: {word['word']}")
```

### Prompt-Guided Transcription

```python
# Provide context to improve accuracy
result = model.transcribe(
    audio,
    initial_prompt="Robot commands: pick, place, navigate, inspect, wait."
)
```

## Performance Optimization

### GPU Acceleration

```python
import torch

# Force GPU usage
model = whisper.load_model("base", device="cuda")

# Or automatically select device
device = "cuda" if torch.cuda.is_available() else "cpu"
model = whisper.load_model("base", device=device)
```

### Batch Processing

```python
# Process multiple audio files
audio_files = ["cmd1.wav", "cmd2.wav", "cmd3.wav"]

for audio_file in audio_files:
    result = model.transcribe(audio_file)
    print(result["text"])
```

### Model Caching

```python
# First load downloads model (~100MB for base)
model = whisper.load_model("base")  # Downloads to ~/.cache/whisper

# Subsequent loads are instant (cached)
model = whisper.load_model("base")  # Uses cache
```

## Noise Reduction

### Using noisereduce Library

```python
import noisereduce as nr
import soundfile as sf

# Load audio
audio, sr = sf.read("noisy_command.wav")

# Reduce noise
reduced_audio = nr.reduce_noise(
    y=audio,
    sr=sr,
    stationary=True
)

# Transcribe cleaned audio
result = model.transcribe(reduced_audio)
```

## Voice Activity Detection (VAD)

### Using webrtcvad

```python
import webrtcvad
import numpy as np

vad = webrtcvad.Vad(3)  # Aggressiveness (0-3)

def is_speech(audio_frame, sample_rate=16000):
    """
    Check if audio frame contains speech.

    Args:
        audio_frame: Audio samples (10/20/30ms frames)
        sample_rate: Sample rate (8000, 16000, 32000, 48000)

    Returns:
        True if speech detected
    """
    # Convert to 16-bit PCM
    audio_int16 = (audio_frame * 32767).astype(np.int16)

    return vad.is_speech(audio_int16.tobytes(), sample_rate)
```

## Error Handling

```python
from src.vla_core.services.voice_service import VoiceService

try:
    voice_service = VoiceService(model_size="base")
    command = voice_service.transcribe_audio_file("command.wav")

    if command.confidence < 0.5:
        print("Low confidence, please repeat")
    else:
        print(f"Command: {command.transcribed_text}")

except FileNotFoundError:
    print("Audio file not found")
except RuntimeError as e:
    print(f"Whisper error: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
```

## Testing

### Unit Tests

```python
import unittest
from src.vla_core.services.voice_service import VoiceService

class TestVoiceService(unittest.TestCase):
    def setUp(self):
        self.voice_service = VoiceService(model_size="tiny")  # Fast for testing

    def test_transcribe_file(self):
        command = self.voice_service.transcribe_audio_file("tests/data/test_audio.wav")
        self.assertIsInstance(command.transcribed_text, str)
        self.assertGreater(command.confidence, 0.0)
        self.assertEqual(command.language, "en")

    def test_empty_audio(self):
        # Test with silent audio
        import numpy as np
        silent_audio = np.zeros(16000)  # 1 second of silence

        command = self.voice_service._transcribe_chunk(silent_audio)
        self.assertLess(len(command.transcribed_text), 10)  # Should be minimal/empty
```

## Best Practices

1. **Use Appropriate Model Size**: `base` for real-time, `small` for better accuracy
2. **Enable GPU**: 5-10x faster transcription
3. **Implement VAD**: Avoid transcribing silence
4. **Set Confidence Thresholds**: Reject low-confidence transcriptions (< 0.7)
5. **Handle Audio Errors**: Microphone failures, permission issues
6. **Provide User Feedback**: Visual/audio indicators when listening
7. **Test with Noise**: Verify performance in real deployment environments

## Common Issues

### Issue: Slow transcription

**Solution**: Use GPU and smaller model
```python
model = whisper.load_model("base", device="cuda")
```

### Issue: Poor accuracy

**Solution**: Use larger model and provide prompts
```python
model = whisper.load_model("small")
result = model.transcribe(audio, initial_prompt="Robot commands...")
```

### Issue: No microphone access

**Solution**: Check permissions and device
```python
import sounddevice as sd
print(sd.query_devices())  # List available devices
```

## Next Steps

- **[Chapter 2: Cognitive Planning](./02-cognitive-planning)**: Convert text to robot action plans
- **[Chapter 3: Pure Python Execution](./03-python-execution)**: Execute actions without ROS 2
- **[Chapter 4: End-to-End Pipeline](./04-end-to-end-pipeline)**: Complete VLA system integration

## Additional Resources

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [Whisper Model Card](https://github.com/openai/whisper/blob/main/model-card.md)
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/docs/)
- [Voice Activity Detection](https://github.com/wiseman/py-webrtcvad)
