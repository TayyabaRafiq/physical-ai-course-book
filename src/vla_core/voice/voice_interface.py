"""
VoiceInterface implementation
Coordinates audio capture and transcription for voice command processing
"""

import asyncio
from typing import AsyncIterator, Optional

from ..contracts.interfaces import IVoiceInterface
from ..models.voice_command import VoiceCommand
from ..utils.config import get_config
from ..utils.errors import TranscriptionError
from ..utils.logging_config import get_logger, log_pipeline_event
from .audio_capture import AudioCapture
from .transcription import WhisperTranscriber

logger = get_logger(__name__)


class VoiceInterface(IVoiceInterface):
    """
    Complete voice input processing pipeline.
    Coordinates audio capture and speech-to-text transcription.
    """

    def __init__(
        self,
        audio_capture: Optional[AudioCapture] = None,
        transcriber: Optional[WhisperTranscriber] = None
    ):
        self.config = get_config()
        self.audio_capture = audio_capture or AudioCapture()
        self.transcriber = transcriber or WhisperTranscriber()
        self._confidence_threshold = self.config.whisper_confidence_threshold
        self._max_retries = self.config.transcription_retry_attempts
        self._listening = False

    async def initialize(self) -> None:
        """Load Whisper model and initialize audio capture."""
        logger.info("Initializing voice interface...")
        await self.transcriber.load_model()
        logger.info("Voice interface initialized")

    async def listen(self, timeout: Optional[float] = None) -> VoiceCommand:
        """
        Listen for a voice command with optional timeout.

        Args:
            timeout: Maximum time to wait for command (seconds)

        Returns:
            VoiceCommand with transcribed text
        """
        if timeout:
            try:
                return await asyncio.wait_for(
                    self.listen_for_command(),
                    timeout=timeout
                )
            except asyncio.TimeoutError:
                raise TranscriptionError(
                    f"Voice input timeout after {timeout}s",
                    confidence=0.0,
                    error_code="VOICE_TIMEOUT",
                    recoverable=True
                )
        else:
            return await self.listen_for_command()

    async def stop_listening(self) -> None:
        """Stop listening for voice commands."""
        self._listening = False
        logger.info("Voice listening stopped")

    def is_ready(self) -> bool:
        """Check if voice interface is ready."""
        return self.transcriber.is_model_loaded()

    async def listen_for_command(self) -> VoiceCommand:
        if not self.transcriber.is_model_loaded():
            logger.info("Loading Whisper model...")
            await self.transcriber.load_model()

        attempt = 0
        last_error = None

        while attempt <= self._max_retries:
            try:
                logger.info(f"Listening for voice command (attempt {attempt + 1}/{self._max_retries + 1})...")
                
                log_pipeline_event(logger, "audio_capture_start", "voice", attempt=attempt + 1)

                max_duration = self.config.audio_max_duration
                audio_buffer = await self.audio_capture.record_fixed_duration(
                    duration_seconds=min(10.0, max_duration)
                )

                log_pipeline_event(logger, "transcription_start", "voice", buffer_size=len(audio_buffer))

                transcribed_text, confidence = await self.transcriber.transcribe(audio_buffer)

                log_pipeline_event(logger, "transcription_complete", "voice", text=transcribed_text, confidence=confidence)

                command = VoiceCommand(
                    audio_buffer=audio_buffer if self.config.log_execution_traces else None,
                    transcribed_text=transcribed_text,
                    confidence=confidence,
                    language="en",
                    sample_rate=self.config.audio_sample_rate
                )

                if command.is_high_confidence(self._confidence_threshold):
                    logger.info(f"Command captured successfully", text=command.transcribed_text, confidence=command.confidence)
                    return command
                else:
                    logger.warning(f"Low confidence: {confidence:.2f} < {self._confidence_threshold}", text=transcribed_text)

                    if attempt < self._max_retries:
                        logger.info("Requesting retry due to low confidence")
                        await self._request_user_retry(f"I heard: '{transcribed_text}'. Please repeat if incorrect.")
                        attempt += 1
                        continue
                    else:
                        logger.warning("Returning low-confidence result after max retries")
                        return command

            except Exception as e:
                last_error = e
                logger.error(f"Error during voice capture/transcription: {e}")
                
                if attempt < self._max_retries:
                    logger.info(f"Retrying... ({attempt + 1}/{self._max_retries})")
                    attempt += 1
                    await asyncio.sleep(1.0)
                else:
                    break

        raise TranscriptionError(
            f"Failed to capture voice command after {self._max_retries + 1} attempts",
            confidence=0.0,
            error_code="MAX_RETRIES_EXCEEDED",
            recoverable=False
        ) from last_error

    async def start_continuous_listening(self) -> AsyncIterator[VoiceCommand]:
        logger.info("Starting continuous listening mode")

        if not self.transcriber.is_model_loaded():
            await self.transcriber.load_model()

        try:
            while True:
                try:
                    command = await self.listen_for_command()
                    yield command
                except TranscriptionError as e:
                    if not e.recoverable:
                        logger.error(f"Unrecoverable transcription error: {e}")
                        break
                    else:
                        logger.warning(f"Recoverable error, continuing: {e}")
                        await asyncio.sleep(1.0)
                        continue
                except KeyboardInterrupt:
                    logger.info("Continuous listening interrupted by user")
                    break
        finally:
            logger.info("Continuous listening stopped")

    def set_confidence_threshold(self, threshold: float) -> None:
        if not 0.0 <= threshold <= 1.0:
            raise ValueError(f"Threshold must be in [0.0, 1.0], got {threshold}")
        self._confidence_threshold = threshold
        logger.info(f"Confidence threshold set to {threshold}")

    async def _request_user_retry(self, message: str) -> None:
        logger.info(f"User feedback: {message}")
        await asyncio.sleep(0.5)

    def get_audio_devices(self) -> list:
        return self.audio_capture.get_available_devices()
