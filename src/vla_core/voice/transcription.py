"""
WhisperTranscriber implementation
Speech-to-text using OpenAI Whisper model
"""

import asyncio
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import whisper

from ..contracts.interfaces import ITranscriber
from ..utils.config import get_config
from ..utils.errors import TranscriptionError
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class WhisperTranscriber(ITranscriber):
    """
    Speech-to-text transcriber using OpenAI Whisper.
    Supports local model inference with optional GPU acceleration.
    """

    def __init__(self):
        """Initialize transcriber with configuration."""
        self.config = get_config()
        self._model: Optional[whisper.Whisper] = None
        self._model_name = self.config.whisper_model
        self._use_gpu = self.config.whisper_use_gpu
        self._device = "cuda" if self._use_gpu else "cpu"

    async def load_model(self) -> None:
        """Load Whisper model into memory."""
        if self._model is not None:
            logger.info("Whisper model already loaded")
            return

        try:
            logger.info(f"Loading Whisper model: {self._model_name} on {self._device}")
            
            # Load model in executor to avoid blocking
            self._model = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: whisper.load_model(self._model_name, device=self._device)
            )
            
            logger.info(f"Whisper model {self._model_name} loaded successfully")

        except Exception as e:
            raise TranscriptionError(
                f"Failed to load Whisper model '{self._model_name}': {str(e)}",
                error_code="MODEL_LOAD_FAILED"
            )

    def is_model_loaded(self) -> bool:
        """Check if model is ready."""
        return self._model is not None

    async def transcribe(self, audio_buffer: bytes) -> Tuple[str, float]:
        """
        Transcribe audio to text.
        
        Args:
            audio_buffer: WAV-formatted audio data
            
        Returns:
            Tuple of (transcribed_text, confidence_score)
        """
        if not self.is_model_loaded():
            await self.load_model()

        try:
            # Convert audio buffer to numpy array
            audio_array = self._audio_bytes_to_array(audio_buffer)

            # Run transcription in executor (CPU/GPU intensive)
            result = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self._model.transcribe(
                    audio_array,
                    language="en",
                    fp16=self._use_gpu  # Use FP16 on GPU for speed
                )
            )

            text = result["text"].strip()
            
            # Whisper doesn't provide confidence directly, estimate from segments
            confidence = self._estimate_confidence(result)

            logger.info(
                f"Transcription complete",
                text=text,
                confidence=confidence,
                language=result.get("language", "en")
            )

            return text, confidence

        except Exception as e:
            raise TranscriptionError(
                f"Transcription failed: {str(e)}",
                error_code="TRANSCRIPTION_FAILED"
            )

    def _audio_bytes_to_array(self, audio_bytes: bytes) -> np.ndarray:
        """
        Convert audio bytes to numpy array for Whisper.
        
        Args:
            audio_bytes: WAV-formatted audio data
            
        Returns:
            Float32 numpy array normalized to [-1, 1]
        """
        import wave
        from io import BytesIO

        try:
            # Read WAV data
            with wave.open(BytesIO(audio_bytes), 'rb') as wav_file:
                sample_rate = wav_file.getframerate()
                n_channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                frames = wav_file.readframes(wav_file.getnframes())

            # Convert to numpy array
            audio_array = np.frombuffer(frames, dtype=np.int16)

            # Convert to mono if stereo
            if n_channels == 2:
                audio_array = audio_array.reshape(-1, 2).mean(axis=1)

            # Normalize to [-1, 1] float32
            audio_array = audio_array.astype(np.float32) / 32768.0

            # Resample to 16kHz if needed (Whisper expects 16kHz)
            if sample_rate != 16000:
                logger.warning(f"Resampling from {sample_rate}Hz to 16000Hz")
                audio_array = self._resample_audio(audio_array, sample_rate, 16000)

            return audio_array

        except Exception as e:
            raise TranscriptionError(
                f"Failed to convert audio to array: {str(e)}",
                error_code="AUDIO_CONVERSION_FAILED"
            )

    def _resample_audio(
        self,
        audio: np.ndarray,
        orig_sr: int,
        target_sr: int
    ) -> np.ndarray:
        """
        Simple linear resampling of audio.
        
        Args:
            audio: Input audio array
            orig_sr: Original sample rate
            target_sr: Target sample rate
            
        Returns:
            Resampled audio array
        """
        # Calculate resampling ratio
        ratio = target_sr / orig_sr
        new_length = int(len(audio) * ratio)

        # Simple linear interpolation
        indices = np.linspace(0, len(audio) - 1, new_length)
        resampled = np.interp(indices, np.arange(len(audio)), audio)

        return resampled.astype(np.float32)

    def _estimate_confidence(self, result: dict) -> float:
        """
        Estimate transcription confidence from Whisper result.
        
        Whisper doesn't provide explicit confidence, so we estimate from:
        - Average log probability of tokens
        - No speech probability
        
        Args:
            result: Whisper transcription result
            
        Returns:
            Confidence score [0.0, 1.0]
        """
        # Get segments with probabilities
        segments = result.get("segments", [])
        
        if not segments:
            return 0.5  # Default if no segments

        # Calculate average confidence from segment probabilities
        confidences = []
        for segment in segments:
            # Whisper provides avg_logprob per segment
            avg_logprob = segment.get("avg_logprob", -1.0)
            # Convert log prob to probability (exponential)
            prob = np.exp(avg_logprob)
            # Clamp to [0, 1]
            prob = max(0.0, min(1.0, prob))
            confidences.append(prob)

        # Weight by segment length
        total_confidence = np.mean(confidences) if confidences else 0.5

        # Penalize for no_speech_prob
        no_speech_prob = result.get("no_speech_prob", 0.0)
        total_confidence *= (1.0 - no_speech_prob)

        return float(total_confidence)

    async def transcribe_file(self, file_path: Path) -> Tuple[str, float]:
        """
        Transcribe audio from file.
        
        Args:
            file_path: Path to audio file
            
        Returns:
            Tuple of (transcribed_text, confidence_score)
        """
        try:
            with open(file_path, 'rb') as f:
                audio_bytes = f.read()
            
            return await self.transcribe(audio_bytes)

        except Exception as e:
            raise TranscriptionError(
                f"Failed to transcribe file '{file_path}': {str(e)}",
                error_code="FILE_TRANSCRIPTION_FAILED"
            )
