"""
Main VLA Pipeline coordinating voice→cognition→execution flow.

This is the central orchestrator for the complete voice-to-action system.
"""

import asyncio
from typing import Optional

from ..cognition import ActionPlanner, IntentParser, PlanValidator
from ..contracts.interfaces import IVlaPipeline
from ..execution import ActionService, ExecutionMonitor
from ..models.execution_log import ExecutionLog
from ..utils.config import get_config
from ..utils.errors import VlaError
from ..utils.logging_config import PipelineLogger, setup_logging
from ..voice import VoiceInterface

from .state_manager import StateManager

logger = PipelineLogger(stage="pipeline")


class VlaPipeline(IVlaPipeline):
    """
    Main VLA pipeline orchestrating the complete voice-to-action flow.

    Pipeline stages:
        1. Voice: Capture audio and transcribe to text
        2. Cognition: Parse intent and generate action plan
        3. Validation: Check plan safety and feasibility
        4. Execution: Execute plan on robot with monitoring
        5. Logging: Record complete trace

    Features:
        - End-to-end async processing
        - State persistence
        - Error recovery
        - Execution logs
        - Emergency stop support
    """

    def __init__(
        self,
        voice_interface: Optional[VoiceInterface] = None,
        intent_parser: Optional[IntentParser] = None,
        action_planner: Optional[ActionPlanner] = None,
        plan_validator: Optional[PlanValidator] = None,
        execution_monitor: Optional[ExecutionMonitor] = None,
        state_manager: Optional[StateManager] = None,
    ):
        """
        Initialize VLA pipeline.

        Args:
            voice_interface: Voice interface (None = create default)
            intent_parser: Intent parser (None = create default)
            action_planner: Action planner (None = create default)
            plan_validator: Plan validator (None = create default)
            execution_monitor: Execution monitor (None = create default)
            state_manager: State manager (None = create default)
        """
        self.config = get_config()

        # Initialize components
        self.voice_interface = voice_interface or VoiceInterface()
        self.intent_parser = intent_parser or IntentParser()
        self.action_planner = action_planner or ActionPlanner()
        self.plan_validator = plan_validator or PlanValidator()
        self.execution_monitor = execution_monitor or ExecutionMonitor()
        self.state_manager = state_manager or StateManager()

        self._initialized = False

        logger.info("VlaPipeline created")

    async def initialize(self) -> None:
        """Initialize all pipeline components."""
        if self._initialized:
            return

        logger.info("Initializing VLA pipeline...")

        try:
            # Initialize voice interface (loads Whisper model)
            await self.voice_interface.initialize()

            # Initialize cognitive components (load prompts)
            await self.intent_parser.initialize()
            await self.action_planner.initialize()

            self._initialized = True
            logger.info("VLA pipeline initialized successfully")

        except Exception as e:
            logger.error("Pipeline initialization failed", error=str(e))
            raise VlaError(f"Pipeline initialization failed: {e}")

    async def process_voice_command(
        self, timeout: Optional[float] = None
    ) -> ExecutionLog:
        """
        Process a complete voice command through the pipeline.

        Args:
            timeout: Maximum processing time for voice input

        Returns:
            Execution log with complete trace

        Raises:
            VlaError: If processing fails
        """
        if not self._initialized:
            await self.initialize()

        logger.info("Processing voice command...")
        pipeline_start = asyncio.get_event_loop().time()

        try:
            # Stage 1: Voice Input
            logger.start("voice_stage")
            voice_start = asyncio.get_event_loop().time()

            voice_command = await self.voice_interface.listen(timeout)
            self.state_manager.store_voice_command(voice_command)

            voice_duration = (asyncio.get_event_loop().time() - voice_start) * 1000
            logger.complete(
                "voice_stage",
                duration_ms=voice_duration,
                text_length=len(voice_command.transcribed_text),
                confidence=voice_command.confidence,
            )

            logger.info(
                f"Voice command: '{voice_command.transcribed_text}'",
                confidence=voice_command.confidence,
            )

            # Stage 2: Intent Parsing
            logger.start("cognition_stage_parse")
            parse_start = asyncio.get_event_loop().time()

            parsed_intent = await self.intent_parser.parse(voice_command)
            self.state_manager.store_intent(parsed_intent)

            parse_duration = (asyncio.get_event_loop().time() - parse_start) * 1000
            logger.complete(
                "cognition_stage_parse",
                duration_ms=parse_duration,
                action_type=parsed_intent.action_type.value,
            )

            # Check for clarification
            if parsed_intent.requires_clarification:
                logger.warning(
                    "Command requires clarification",
                    ambiguities=parsed_intent.ambiguities,
                )
                # In production, this would trigger a dialogue
                # For MVP, we'll proceed anyway with a warning

            # Stage 3: Action Planning
            logger.start("cognition_stage_plan")
            plan_start = asyncio.get_event_loop().time()

            action_plan = await self.action_planner.generate_plan(parsed_intent)
            self.state_manager.store_plan(action_plan)

            plan_duration = (asyncio.get_event_loop().time() - plan_start) * 1000
            logger.complete(
                "cognition_stage_plan",
                duration_ms=plan_duration,
                num_steps=len(action_plan.steps),
            )

            # Stage 4: Plan Validation
            logger.start("validation_stage")
            validation_start = asyncio.get_event_loop().time()

            is_valid, errors = await self.plan_validator.validate(action_plan)

            validation_duration = (asyncio.get_event_loop().time() - validation_start) * 1000
            logger.complete(
                "validation_stage",
                duration_ms=validation_duration,
                is_valid=is_valid,
                num_errors=len(errors),
            )

            if not is_valid:
                error_msg = "; ".join(errors)
                logger.error(
                    "Plan validation failed",
                    Exception(error_msg),
                    errors=errors,
                )
                raise VlaError(f"Plan validation failed: {error_msg}")

            # Stage 5: Execution
            logger.start("execution_stage")
            execution_start = asyncio.get_event_loop().time()

            execution_state = await self.execution_monitor.execute_plan(action_plan)

            execution_duration = (asyncio.get_event_loop().time() - execution_start) * 1000
            logger.complete(
                "execution_stage",
                duration_ms=execution_duration,
                status=execution_state.status.value,
            )

            # Get execution log
            execution_log = self.execution_monitor.get_current_log()

            if execution_log:
                # Fill in missing fields
                execution_log.voice_command_text = voice_command.transcribed_text
                execution_log.parsed_intent_summary = (
                    f"{parsed_intent.action_type.value}: "
                    f"{', '.join(obj.name for obj in parsed_intent.target_objects)}"
                )

            # Calculate total pipeline time
            total_duration = (asyncio.get_event_loop().time() - pipeline_start) * 1000

            # Check latency budget
            if total_duration > self.config.latency_budget * 1000:
                logger.warning(
                    "Latency budget exceeded",
                    actual_ms=total_duration,
                    budget_ms=self.config.latency_budget * 1000,
                )

            logger.metric("pipeline_latency", total_duration, unit="ms")
            logger.info(
                f"Pipeline complete in {total_duration:.0f}ms",
                status=execution_state.status.value,
            )

            return execution_log

        except VlaError:
            raise
        except Exception as e:
            logger.error("Pipeline processing failed", error=str(e))
            raise VlaError(f"Pipeline processing failed: {e}")

    async def emergency_stop(self) -> bool:
        """
        Immediately stop all execution.

        Returns:
            True if successfully stopped
        """
        logger.warning("Emergency stop requested")

        try:
            # Cancel current execution
            await self.execution_monitor.cancel_execution()

            # Stop listening
            await self.voice_interface.stop_listening()

            logger.info("Emergency stop completed")
            return True

        except Exception as e:
            logger.error("Emergency stop failed", error=str(e))
            return False

    def is_ready(self) -> bool:
        """Check if pipeline is ready to process commands."""
        return self._initialized and self.voice_interface.is_ready()


# Example usage
if __name__ == "__main__":
    import sys

    async def test_vla_pipeline():
        """Test VLA pipeline."""
        # Setup logging
        setup_logging(log_level="INFO", enable_json=False)

        # Create pipeline
        pipeline = VlaPipeline()

        print("Initializing pipeline...")
        await pipeline.initialize()

        print(f"Pipeline ready: {pipeline.is_ready()}")
        print("\nSpeak a command (timeout: 5 seconds)...")

        try:
            # Process command
            log = await pipeline.process_voice_command(timeout=5.0)

            print(f"\n{'='*60}")
            print("EXECUTION LOG")
            print(f"{'='*60}")
            print(f"Command: {log.voice_command_text}")
            print(f"Intent: {log.parsed_intent_summary}")
            print(f"Steps: {len(log.plan_steps_summary)}")
            for i, step in enumerate(log.plan_steps_summary):
                print(f"  {i+1}. {step}")
            print(f"Status: {log.final_status.value}")
            print(f"Duration: {log.total_duration:.2f}s")
            print(f"Events: {len(log.execution_trace)}")

        except VlaError as e:
            print(f"\nError: {e}")

        finally:
            await pipeline.emergency_stop()

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_vla_pipeline())