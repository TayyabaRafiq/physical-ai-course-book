"""End-to-end pipeline integration tests."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from src.vla_core.pipeline.vla_pipeline import VlaPipeline
from src.vla_core.models import ExecutionStatus
from src.vla_core.utils.errors import VlaError


@pytest.mark.asyncio
class TestPipelineEndToEnd:
    """Test complete pipeline flow."""

    async def test_full_pipeline_success(self, mock_config):
        """Test complete successful pipeline execution."""
        pipeline = VlaPipeline()

        # Mock voice interface
        pipeline.voice_interface.listen = AsyncMock(
            return_value=MagicMock(
                transcribed_text="Pick up the red block",
                confidence=0.95,
                audio_buffer=b"audio",
                id=MagicMock(),
            )
        )
        pipeline.voice_interface.is_ready = MagicMock(return_value=True)

        # Mock intent parser
        pipeline.intent_parser.parse = AsyncMock(
            return_value=MagicMock(
                command_id=MagicMock(),
                action_type="pick",
                target_objects=[],
                parameters={"object_id": "red_block_1"},
                ambiguities=[],
                requires_clarification=False,
            )
        )

        # Mock action planner
        pipeline.action_planner.generate_plan = AsyncMock(
            return_value=MagicMock(
                plan_id=MagicMock(),
                intent_id=MagicMock(),
                steps=[MagicMock()],
                preconditions=["Ready"],
                expected_outcomes=["Done"],
                validated=False,
            )
        )

        # Mock plan validator
        pipeline.plan_validator.validate = AsyncMock(return_value=(True, []))

        # Mock execution monitor
        pipeline.execution_monitor.execute_plan = AsyncMock(
            return_value=MagicMock(status=ExecutionStatus.COMPLETED)
        )
        pipeline.execution_monitor.get_current_log = MagicMock(
            return_value=MagicMock(
                voice_command_text="Pick up the red block",
                parsed_intent_summary="pick: red_block_1",
                plan_steps_summary=["Navigate", "Pick"],
                final_status=ExecutionStatus.COMPLETED,
                total_duration=5.0,
                execution_trace=[],
            )
        )

        # Initialize
        await pipeline.initialize()

        # Process command
        log = await pipeline.process_voice_command(timeout=5.0)

        # Verify
        assert log is not None
        assert log.final_status == ExecutionStatus.COMPLETED

    async def test_pipeline_validation_failure(self, mock_config):
        """Test pipeline with plan validation failure."""
        pipeline = VlaPipeline()

        # Mock components
        pipeline.voice_interface.listen = AsyncMock(
            return_value=MagicMock(
                transcribed_text="Invalid command",
                confidence=0.95,
                audio_buffer=b"audio",
                id=MagicMock(),
            )
        )
        pipeline.voice_interface.is_ready = MagicMock(return_value=True)

        pipeline.intent_parser.parse = AsyncMock(
            return_value=MagicMock(
                command_id=MagicMock(),
                action_type="pick",
                target_objects=[],
                parameters={},
                ambiguities=[],
                requires_clarification=False,
            )
        )

        pipeline.action_planner.generate_plan = AsyncMock(
            return_value=MagicMock(
                plan_id=MagicMock(),
                steps=[],  # Empty plan
                preconditions=[],
                expected_outcomes=[],
                validated=False,
            )
        )

        # Mock validator to fail
        pipeline.plan_validator.validate = AsyncMock(
            return_value=(False, ["Plan has no steps"])
        )

        await pipeline.initialize()

        # Should raise VlaError
        with pytest.raises(VlaError) as exc_info:
            await pipeline.process_voice_command(timeout=5.0)

        assert "validation failed" in str(exc_info.value).lower()

    async def test_pipeline_initialization(self):
        """Test pipeline initialization."""
        pipeline = VlaPipeline()

        assert pipeline.is_ready() is False

        # Mock initializations
        pipeline.voice_interface.initialize = AsyncMock()
        pipeline.intent_parser.initialize = AsyncMock()
        pipeline.action_planner.initialize = AsyncMock()

        await pipeline.initialize()

        # Verify components initialized
        pipeline.voice_interface.initialize.assert_called_once()
        pipeline.intent_parser.initialize.assert_called_once()
        pipeline.action_planner.initialize.assert_called_once()

    async def test_emergency_stop(self):
        """Test emergency stop functionality."""
        pipeline = VlaPipeline()

        # Mock components
        pipeline.execution_monitor.cancel_execution = AsyncMock(return_value=True)
        pipeline.voice_interface.stop_listening = AsyncMock()

        result = await pipeline.emergency_stop()

        assert result is True
        pipeline.execution_monitor.cancel_execution.assert_called_once()
        pipeline.voice_interface.stop_listening.assert_called_once()


@pytest.mark.asyncio
class TestStateManager:
    """Test state manager."""

    async def test_store_and_retrieve_voice_command(self, sample_voice_command):
        """Test storing and retrieving voice command."""
        from src.vla_core.pipeline.state_manager import StateManager

        manager = StateManager()

        manager.store_voice_command(sample_voice_command)
        retrieved = manager.get_voice_command(sample_voice_command.id)

        assert retrieved is not None
        assert retrieved.id == sample_voice_command.id
        assert retrieved.transcribed_text == sample_voice_command.transcribed_text

    async def test_execution_state_tracking(self, sample_action_plan):
        """Test execution state tracking."""
        from src.vla_core.pipeline.state_manager import StateManager

        manager = StateManager()
        manager.store_plan(sample_action_plan)

        # Create execution state
        state = manager.create_execution_state(sample_action_plan.plan_id)

        assert state is not None
        assert state.plan_id == sample_action_plan.plan_id

        # Should be marked as current
        assert manager.is_execution_in_progress() is True

    async def test_state_manager_clear(self):
        """Test clearing state manager."""
        from src.vla_core.pipeline.state_manager import StateManager

        manager = StateManager()

        # Add some state
        from src.vla_core.models.voice_command import VoiceCommand

        cmd = VoiceCommand(
            audio_buffer=b"test",
            transcribed_text="Test",
            confidence=0.9,
        )
        manager.store_voice_command(cmd)

        assert len(manager._voice_commands) > 0

        # Clear all
        manager.clear_all()

        assert len(manager._voice_commands) == 0
        assert manager.is_execution_in_progress() is False