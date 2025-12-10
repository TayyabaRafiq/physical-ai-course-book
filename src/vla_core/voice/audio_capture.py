"""
AudioCapture implementation using PyAudio
Captures audio from microphone for voice command processing
"""

import asyncio
import wave
from io import BytesIO
from typing import AsyncIterator, Optional

import numpy as np

# Mock pyaudio for environments without audio hardware (e.g., static site deployment)
try:
    import pyaudio
except ImportError:
    # Create mock pyaudio module for import compatibility
    class MockPyAudio:
        paInt16 = 8  # Mock format constant

        class PyAudio:
            def __init__(self):
                pass
            def get_default_input_device_info(self):
                return {'index': 0, 'name': 'Mock Device'}
            def get_device_info_by_index(self, index):
                return {'name': 'Mock Device', 'maxInputChannels': 1, 'defaultSampleRate': 16000}
            def open(self, *args, **kwargs):
                return None
            def terminate(self):
                pass
            def get_device_count(self):
                return 0

        class Stream:
            def read(self, frames):
                return b'\x00' * frames * 2  # Mock audio data
            def stop_stream(self):
                pass
            def close(self):
                pass

    pyaudio = MockPyAudio()

from ..contracts.interfaces import IAudioCapture
from ..utils.config import get_config
from ..utils.errors import AudioCaptureError
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class AudioCapture(IAudioCapture):
    """
    Audio capture implementation using PyAudio.
    Captures microphone input for speech recognition.
    """

    def __init__(self):
        """Initialize audio capture with configuration."""
        self.config = get_config()
        self._audio: Optional[pyaudio.PyAudio] = None
        self._stream: Optional[pyaudio.Stream] = None
        self._is_recording = False
        self._audio_buffer: list = []
        self._sample_rate = self.config.audio_sample_rate
        self._chunk_size = self.config.audio_chunk_size
        self._device_index = self.config.audio_device_index

    async def start_recording(self) -> None:
        """Start audio capture from microphone."""
        if self._is_recording:
            logger.warning("Already recording, ignoring start_recording call")
            return

        try:
            # Initialize PyAudio
            self._audio = pyaudio.PyAudio()

            # Get device info
            if self._device_index == -1:
                device_info = self._audio.get_default_input_device_info()
                self._device_index = device_info['index']
                logger.info(f"Using default audio device: {device_info['name']}")
            else:
                device_info = self._audio.get_device_info_by_index(self._device_index)
                logger.info(f"Using audio device {self._device_index}: {device_info['name']}")

            # Open stream
            self._stream = self._audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self._sample_rate,
                input=True,
                input_device_index=self._device_index,
                frames_per_buffer=self._chunk_size
            )

            self._is_recording = True
            self._audio_buffer = []
            logger.info("Audio recording started")

        except Exception as e:
            raise AudioCaptureError(
                f"Failed to start audio recording: {str(e)}",
                device_index=self._device_index
            )

    async def stop_recording(self) -> bytes:
        """
        Stop recording and return audio buffer.
        
        Returns:
            WAV-formatted audio data as bytes
        """
        if not self._is_recording:
            raise AudioCaptureError("Not currently recording")

        try:
            # Stop and close stream
            if self._stream:
                self._stream.stop_stream()
                self._stream.close()

            # Terminate PyAudio
            if self._audio:
                self._audio.terminate()

            self._is_recording = False
            logger.info("Audio recording stopped")

            # Convert buffer to WAV format
            audio_data = b''.join(self._audio_buffer)
            wav_buffer = self._create_wav_buffer(audio_data)

            return wav_buffer

        except Exception as e:
            raise AudioCaptureError(f"Failed to stop recording: {str(e)}")

    async def get_audio_stream(self) -> AsyncIterator[bytes]:
        """
        Get continuous audio stream for real-time processing.
        Yields audio chunks as they're captured.
        """
        if not self._is_recording:
            await self.start_recording()

        try:
            while self._is_recording and self._stream:
                # Read chunk from stream (blocking call)
                data = await asyncio.get_event_loop().run_in_executor(
                    None,
                    self._stream.read,
                    self._chunk_size
                )
                self._audio_buffer.append(data)
                yield data

        except Exception as e:
            logger.error(f"Error in audio stream: {e}")
            raise AudioCaptureError(f"Audio stream error: {str(e)}")

    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._is_recording

    def _create_wav_buffer(self, audio_data: bytes) -> bytes:
        """
        Create WAV-formatted buffer from raw audio data.
        
        Args:
            audio_data: Raw PCM audio data
            
        Returns:
            WAV-formatted bytes
        """
        buffer = BytesIO()
        
        with wave.open(buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(self._sample_rate)
            wav_file.writeframes(audio_data)

        return buffer.getvalue()

    async def record_fixed_duration(self, duration_seconds: float) -> bytes:
        """
        Record audio for a fixed duration.
        
        Args:
            duration_seconds: Recording duration
            
        Returns:
            WAV-formatted audio data
        """
        await self.start_recording()
        
        # Calculate number of chunks to record
        num_chunks = int(self._sample_rate / self._chunk_size * duration_seconds)
        
        try:
            for _ in range(num_chunks):
                if self._stream:
                    data = await asyncio.get_event_loop().run_in_executor(
                        None,
                        self._stream.read,
                        self._chunk_size
                    )
                    self._audio_buffer.append(data)
        finally:
            return await self.stop_recording()

    def get_available_devices(self) -> list:
        """
        Get list of available audio input devices.

        Returns:
            List of device info dictionaries
        """
        if not self._audio:
            self._audio = pyaudio.PyAudio()

        devices = []
        for i in range(self._audio.get_device_count()):
            info = self._audio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                devices.append({
                    'index': i,
                    'name': info['name'],
                    'channels': info['maxInputChannels'],
                    'sample_rate': int(info['defaultSampleRate'])
                })

        return devices

    def list_devices(self) -> list:
        """Alias for get_available_devices() for CLI compatibility."""
        return self.get_available_devices()
