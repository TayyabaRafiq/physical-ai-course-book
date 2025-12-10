"""Tests for voice interface."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from src.vla_core.voice.voice_interface import VoiceInterface
from src.vla_core.models.voice_command import VoiceCommand
from src.vla_core.utils.errors import TranscriptionError


@pytest.mark.asyncio
class TestVoiceInterface:
    """Test VoiceInterface."""

    async def test_initialization(self):
        """Test voice interface initialization."""
        interface = VoiceInterface()

        assert interface.audio_capture is not None
        assert interface.transcriber is not None

    async def test_initialize_loads_model(self):
        """Test initialization loads Whisper model."""
        interface = VoiceInterface()

        # Mock transcriber
        interface.transcriber.load_model = AsyncMock()

        await interface.initialize()

        interface.transcriber.load_model.assert_called_once()

    async def test_listen_success(self, mock_config):
        """Test successful voice command listening."""
        interface = VoiceInterface()

        # Mock components
        interface.audio_capture.record_fixed_duration = AsyncMock(
            return_value=b"audio_data"
        )
        interface.transcriber.transcribe = AsyncMock(
            return_value=("Pick up the red block", 0.95)
        )
        interface.transcriber.is_ready = MagicMock(return_value=True)

        # Listen
        command = await interface.listen(timeout=5.0)

        assert isinstance(command, VoiceCommand)
        assert command.transcribed_text == "Pick up the red block"
        assert command.confidence == 0.95

    async def test_listen_low_confidence_retry(self, mock_config):
        """Test retry on low confidence."""
        interface = VoiceInterface()

        # Mock low confidence first, then high
        interface.audio_capture.record_fixed_duration = AsyncMock(
            return_value=b"audio_data"
        )
        interface.transcriber.transcribe = AsyncMock(
            side_effect=[
                ("unclear", 0.5),  # First attempt - low confidence
                ("Pick up block", 0.95),  # Second attempt - high confidence
            ]
        )
        interface.transcriber.is_ready = MagicMock(return_value=True)

        # Listen
        command = await interface.listen(timeout=5.0)

        # Should have retried
        assert interface.transcriber.transcribe.call_count == 2
        assert command.confidence == 0.95

    async def test_listen_max_retries_exceeded(self, mock_config):
        """Test returning low confidence after max retries."""
        interface = VoiceInterface()

        # Mock always low confidence
        interface.audio_capture.record_fixed_duration = AsyncMock(
            return_value=b"audio_data"
        )
        interface.transcriber.transcribe = AsyncMock(return_value=("unclear", 0.5))
        interface.transcriber.is_ready = MagicMock(return_value=True)

        # Listen - should return low confidence after retries
        command = await interface.listen(timeout=5.0)

        assert command.confidence == 0.5

    async def test_stop_listening(self):
        """Test stopping voice interface."""
        interface = VoiceInterface()
        interface.audio_capture.stop_capture = AsyncMock()

        await interface.stop_listening()

        interface.audio_capture.stop_capture.assert_called_once()

    async def test_is_ready(self):
        """Test ready check."""
        interface = VoiceInterface()

        interface.transcriber.is_ready = MagicMock(return_value=True)
        assert interface.is_ready() is True

        interface.transcriber.is_ready = MagicMock(return_value=False)
        assert interface.is_ready() is False