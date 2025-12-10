"""
Configuration Management for VLA Integration
Loads and validates configuration from environment variables
"""

import os
from pathlib import Path
from typing import Optional

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class VLAConfig(BaseSettings):
    """
    Main configuration class for VLA Integration.
    Loads settings from environment variables and .env file.
    """

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )

    # ========== OpenAI API Configuration ==========
    openai_api_key: str = Field(..., description="OpenAI API key")
    openai_model: str = Field(default="gpt-4-turbo", description="OpenAI model name")
    llm_timeout: float = Field(default=10.0, ge=1.0, le=60.0, description="LLM request timeout (seconds)")

    # ========== Whisper Configuration ==========
    whisper_model: str = Field(default="medium", description="Whisper model size")
    whisper_confidence_threshold: float = Field(default=0.7, ge=0.0, le=1.0)
    whisper_use_gpu: bool = Field(default=True, description="Use GPU for Whisper inference")

    @field_validator("whisper_model")
    @classmethod
    def validate_whisper_model(cls, v: str) -> str:
        valid_models = ["tiny", "base", "small", "medium", "large"]
        if v not in valid_models:
            raise ValueError(f"Whisper model must be one of {valid_models}")
        return v

    # ========== Audio Configuration ==========
    audio_sample_rate: int = Field(default=16000, description="Audio sample rate (Hz)")
    audio_device_index: int = Field(default=-1, description="Audio input device (-1 for default)")
    audio_chunk_size: int = Field(default=1024, ge=256, le=8192)
    audio_max_duration: int = Field(default=60, ge=1, le=300, description="Max recording duration (seconds)")
    vad_threshold: float = Field(default=0.5, ge=0.0, le=1.0, description="Voice activity detection threshold")

    # ========== ROS 2 Configuration ==========
    ros_domain_id: int = Field(default=42, ge=0, le=232, description="ROS Domain ID")
    ros_localhost_only: bool = Field(default=True, description="Restrict ROS to localhost")
    ros_action_timeout: float = Field(default=30.0, ge=1.0, le=300.0, description="ROS action timeout (seconds)")

    # ========== Safety & Validation Configuration ==========
    max_plan_steps: int = Field(default=10, ge=1, le=50, description="Maximum steps in action plan")
    joint_limit_margin: float = Field(default=0.1, ge=0.0, le=0.5, description="Joint limit safety margin")
    max_force_threshold: float = Field(default=50.0, ge=0.0, le=500.0, description="Max force (Newtons)")
    max_velocity: float = Field(default=0.5, ge=0.0, le=5.0, description="Max velocity (m/s or rad/s)")
    enable_collision_checking: bool = Field(default=True)

    # ========== Logging Configuration ==========
    log_level: str = Field(default="INFO", description="Logging level")
    log_dir: Path = Field(default=Path("./logs"), description="Log directory")
    enable_json_logging: bool = Field(default=True)
    log_execution_traces: bool = Field(default=True)

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        v_upper = v.upper()
        if v_upper not in valid_levels:
            raise ValueError(f"Log level must be one of {valid_levels}")
        return v_upper

    # ========== Pipeline Configuration ==========
    latency_budget: float = Field(default=3.0, ge=0.1, le=30.0, description="Target latency (seconds)")
    enable_plan_caching: bool = Field(default=True)
    transcription_retry_attempts: int = Field(default=2, ge=0, le=5)
    llm_retry_attempts: int = Field(default=3, ge=0, le=10)

    # ========== Simulation Configuration ==========
    gazebo_world_file: Path = Field(
        default=Path("src/simulation/worlds/household_world.sdf"),
        description="Gazebo world file path"
    )
    robot_description_file: Path = Field(
        default=Path("src/simulation/robot_description/humanoid.urdf"),
        description="Robot URDF/SDF file path"
    )
    gazebo_gui_enabled: bool = Field(default=True)
    simulation_step_time: float = Field(default=0.001, ge=0.0001, le=0.01)

    # ========== Feature Flags ==========
    enable_multi_step_planning: bool = Field(default=True, description="Enable User Story 2 features")
    enable_interruptions: bool = Field(default=True, description="Enable User Story 3 features")
    enable_clarification: bool = Field(default=True)
    enable_performance_monitoring: bool = Field(default=True)

    def get_log_dir_absolute(self) -> Path:
        """Get absolute path to log directory."""
        return self.log_dir.resolve()

    def ensure_directories(self) -> None:
        """Create necessary directories if they don't exist."""
        self.log_dir.mkdir(parents=True, exist_ok=True)
        (self.log_dir / "executions").mkdir(parents=True, exist_ok=True)

    def to_dict(self) -> dict:
        """Convert config to dictionary."""
        return self.model_dump()


# Global configuration instance
_config: Optional[VLAConfig] = None


def load_config(env_file: Optional[str] = None) -> VLAConfig:
    """
    Load configuration from environment variables and .env file.

    Args:
        env_file: Optional path to .env file (default: ".env" in current directory)

    Returns:
        Loaded VLAConfig instance
    """
    global _config
    
    if env_file:
        os.environ["ENV_FILE"] = env_file
    
    _config = VLAConfig()
    _config.ensure_directories()
    
    return _config


def get_config() -> VLAConfig:
    """
    Get the global configuration instance.

    Returns:
        VLAConfig instance

    Raises:
        RuntimeError: If config hasn't been loaded yet
    """
    global _config
    
    if _config is None:
        # Auto-load config on first access
        return load_config()
    
    return _config


# Example usage
if __name__ == "__main__":
    # Load configuration
    config = load_config()

    print("=== VLA Configuration ===")
    print(f"OpenAI Model: {config.openai_model}")
    print(f"Whisper Model: {config.whisper_model}")
    print(f"Log Level: {config.log_level}")
    print(f"Log Directory: {config.get_log_dir_absolute()}")
    print(f"Max Plan Steps: {config.max_plan_steps}")
    print(f"ROS Domain ID: {config.ros_domain_id}")
    print(f"Feature Flags:")
    print(f"  - Multi-step Planning: {config.enable_multi_step_planning}")
    print(f"  - Interruptions: {config.enable_interruptions}")
    print(f"  - Clarification: {config.enable_clarification}")
    
    # Validate directories created
    assert config.log_dir.exists(), "Log directory not created"
    print("\n✓ Configuration loaded and validated successfully")
