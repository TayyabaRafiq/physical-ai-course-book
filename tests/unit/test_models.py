"""Tests for data models."""

import pytest
from datetime import datetime
from uuid import uuid4

from src.vla_core.models import ActionType, ExecutionStatus
from src.vla_core.models.action_plan import ActionPlan
from src.vla_core.models.execution_state import ExecutionState, ExecutionError
from src.vla_core.models.parsed_intent import ParsedIntent, ObjectReference
from src.vla_core.models.robot_action import RobotAction, ActionConstraints
from src.vla_core.models.voice_command import VoiceCommand


class TestVoiceCommand:
    """Test VoiceCommand model."""

    def test_create_voice_command(self):
        """Test creating voice command."""
        cmd = VoiceCommand(
            audio_buffer=b"test",
            transcribed_text="Test command",
            confidence=0.95,
        )

        assert cmd.transcribed_text == "Test command"
        assert cmd.confidence == 0.95
        assert cmd.language == "en"
        assert cmd.sample_rate == 16000

    def test_voice_command_validation(self):
        """Test voice command validation."""
        # Empty text should fail
        with pytest.raises(ValueError):
            VoiceCommand(
                audio_buffer=b"test",
                transcribed_text="  ",
                confidence=0.95,
            )

        # Invalid sample rate should fail
        with pytest.raises(ValueError):
            VoiceCommand(
                audio_buffer=b"test",
                transcribed_text="Test",
                confidence=0.95,
                sample_rate=12345,
            )

    def test_high_confidence_check(self):
        """Test confidence threshold check."""
        cmd = VoiceCommand(
            audio_buffer=b"test",
            transcribed_text="Test",
            confidence=0.8,
        )

        assert cmd.is_high_confidence(0.7) is True
        assert cmd.is_high_confidence(0.9) is False


class TestParsedIntent:
    """Test ParsedIntent model."""

    def test_create_parsed_intent(self):
        """Test creating parsed intent."""
        intent = ParsedIntent(
            command_id=uuid4(),
            action_type=ActionType.PICK,
            target_objects=[],
            parameters={"grasp_type": "top"},
        )

        assert intent.action_type == ActionType.PICK
        assert intent.requires_clarification is False

    def test_auto_clarification_detection(self):
        """Test automatic clarification detection."""
        # UNKNOWN action should require clarification
        intent = ParsedIntent(
            command_id=uuid4(),
            action_type=ActionType.UNKNOWN,
            target_objects=[],
            parameters={},
        )

        assert intent.requires_clarification is True

        # Ambiguities should require clarification
        intent2 = ParsedIntent(
            command_id=uuid4(),
            action_type=ActionType.PICK,
            target_objects=[],
            parameters={},
            ambiguities=["Missing object"],
        )

        assert intent2.requires_clarification is True

    def test_parameter_validation(self):
        """Test parameter validation for action types."""
        # PICK without target should fail validation
        intent = ParsedIntent(
            command_id=uuid4(),
            action_type=ActionType.PICK,
            target_objects=[],
            parameters={},
        )

        is_valid = intent.validate_parameters_for_action()
        assert is_valid is False
        assert len(intent.ambiguities) > 0


class TestActionPlan:
    """Test ActionPlan model."""

    def test_create_action_plan(self):
        """Test creating action plan."""
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.NAVIGATE,
                    ros_action_name="/navigate",
                    goal_message={},
                    timeout=10.0,
                )
            ],
            preconditions=["Robot ready"],
            expected_outcomes=["Navigation complete"],
        )

        assert len(plan.steps) == 1
        assert plan.validated is False

    def test_duration_calculation(self):
        """Test duration calculation."""
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.NAVIGATE,
                    ros_action_name="/navigate",
                    goal_message={},
                    timeout=10.0,
                ),
                RobotAction(
                    action_type=ActionType.PICK,
                    ros_action_name="/pick",
                    goal_message={},
                    timeout=5.0,
                ),
            ],
            preconditions=["Ready"],
            expected_outcomes=["Done"],
        )

        duration = plan.calculate_estimated_duration()
        assert duration == 15.0

    def test_validation_marking(self):
        """Test validation marking."""
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.PICK,
                    ros_action_name="/pick",
                    goal_message={},
                    timeout=5.0,
                )
            ],
            preconditions=["Ready"],
            expected_outcomes=["Done"],
        )

        assert plan.is_valid() is False

        plan.mark_validated()
        assert plan.is_valid() is True

        plan.add_validation_error("Test error")
        assert plan.is_valid() is False


class TestExecutionState:
    """Test ExecutionState model."""

    def test_create_execution_state(self):
        """Test creating execution state."""
        state = ExecutionState(plan_id=uuid4())

        assert state.status == ExecutionStatus.IDLE
        assert state.progress_percent == 0.0
        assert len(state.completed_steps) == 0

    def test_start_execution(self):
        """Test starting execution."""
        state = ExecutionState(plan_id=uuid4())
        state.start_execution()

        assert state.status == ExecutionStatus.RUNNING
        assert state.started_at is not None

    def test_complete_step(self):
        """Test completing a step."""
        state = ExecutionState(plan_id=uuid4())
        state.start_execution()

        action_id = uuid4()
        state.complete_step(action_id)

        assert len(state.completed_steps) == 1
        assert state.completed_steps[0] == action_id
        assert state.current_step_index == 1

    def test_progress_update(self):
        """Test progress calculation."""
        state = ExecutionState(plan_id=uuid4())
        state.start_execution()

        state.complete_step(uuid4())
        state.update_progress(total_steps=4)

        assert state.progress_percent == 25.0

    def test_pause_resume(self):
        """Test pause and resume."""
        state = ExecutionState(plan_id=uuid4())
        state.start_execution()

        state.pause()
        assert state.status == ExecutionStatus.PAUSED

        state.resume()
        assert state.status == ExecutionStatus.RUNNING

    def test_completion(self):
        """Test marking complete."""
        state = ExecutionState(plan_id=uuid4())
        state.start_execution()
        state.mark_completed()

        assert state.status == ExecutionStatus.COMPLETED
        assert state.progress_percent == 100.0
        assert state.completed_at is not None


class TestRobotAction:
    """Test RobotAction model."""

    def test_create_robot_action(self):
        """Test creating robot action."""
        action = RobotAction(
            action_type=ActionType.PICK,
            ros_action_name="pick_object",
            goal_message={"object_id": "block_1"},
            timeout=10.0,
        )

        # Should auto-add leading slash
        assert action.ros_action_name == "/pick_object"
        assert action.action_type == ActionType.PICK

    def test_default_constraints(self):
        """Test default constraints."""
        action = RobotAction(
            action_type=ActionType.PICK,
            ros_action_name="/pick",
            goal_message={},
            timeout=5.0,
        )

        assert action.constraints.max_force == 50.0
        assert action.constraints.collision_check is True