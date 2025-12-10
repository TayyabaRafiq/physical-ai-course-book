"""Pytest configuration and shared fixtures."""

import asyncio
from pathlib import Path
from uuid import uuid4

import pytest

from src.vla_core.models import ActionType
from src.vla_core.models.action_plan import ActionPlan
from src.vla_core.models.parsed_intent import ParsedIntent
from src.vla_core.models.robot_action import RobotAction
from src.vla_core.models.voice_command import VoiceCommand


@pytest.fixture
def event_loop():
    """Create event loop for async tests."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def sample_voice_command():
    """Create sample voice command."""
    return VoiceCommand(
        id=uuid4(),
        audio_buffer=b"test_audio_data",
        transcribed_text="Pick up the red block",
        confidence=0.95,
        language="en",
        sample_rate=16000,
    )


@pytest.fixture
def sample_parsed_intent():
    """Create sample parsed intent."""
    from src.vla_core.models.parsed_intent import ObjectReference

    return ParsedIntent(
        command_id=uuid4(),
        action_type=ActionType.PICK,
        target_objects=[
            ObjectReference(
                name="red block",
                type="block",
                color="red",
                position=None,
                confidence=0.92,
            )
        ],
        parameters={"grasp_type": "top", "approach_offset": {"z": 0.1}},
        ambiguities=[],
        requires_clarification=False,
    )


@pytest.fixture
def sample_action_plan(sample_parsed_intent):
    """Create sample action plan."""
    return ActionPlan(
        intent_id=sample_parsed_intent.command_id,
        steps=[
            RobotAction(
                action_type=ActionType.NAVIGATE,
                ros_action_name="/navigate_to_point",
                goal_message={"target_pose": {"x": 2.0, "y": 1.0, "theta": 0.0}},
                timeout=10.0,
            ),
            RobotAction(
                action_type=ActionType.PICK,
                ros_action_name="/pick_object",
                goal_message={
                    "object_id": "red_block_1",
                    "grasp_type": "top",
                },
                timeout=8.0,
            ),
        ],
        preconditions=["Robot ready", "Red block visible"],
        expected_outcomes=["Red block grasped"],
        validated=True,
    )


@pytest.fixture
def mock_config(monkeypatch):
    """Mock configuration for tests."""
    from src.vla_core.utils.config import VLAConfig

    config = VLAConfig(
        openai_api_key="test_key",
        whisper_model="tiny",
        log_level="DEBUG",
        audio_sample_rate=16000,
        ros_domain_id=42,
    )

    monkeypatch.setattr("src.vla_core.utils.config._config", config)
    return config


@pytest.fixture
def temp_log_dir(tmp_path):
    """Create temporary log directory."""
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    return log_dir