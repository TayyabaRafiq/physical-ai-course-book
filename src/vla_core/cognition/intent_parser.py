"""
Intent Parser using LLM for natural language understanding.

Converts voice commands to structured ParsedIntent objects.
"""

import asyncio
from pathlib import Path
from typing import Optional

from ..contracts.interfaces import IIntentParser
from ..models.parsed_intent import ParsedIntent
from ..models.voice_command import VoiceCommand
from ..utils.config import get_config
from ..utils.errors import ParsingError
from ..utils.logging_config import PipelineLogger

from .llm_client import LLMClient

logger = PipelineLogger(stage="cognition")


class IntentParser(IIntentParser):
    """
    Parses voice commands to structured intents using LLM.

    Features:
        - LLM-based natural language understanding
        - Structured output with Pydantic validation
        - Clarification detection
        - Parameter extraction
    """

    def __init__(self, llm_client: Optional[LLMClient] = None):
        """
        Initialize intent parser.

        Args:
            llm_client: LLM client instance (None = create default)
        """
        self.config = get_config()
        self.llm_client = llm_client or LLMClient()

        # Load system prompt
        self.system_prompt: Optional[str] = None

        logger.info("IntentParser initialized")

    async def initialize(self) -> None:
        """Load prompt templates."""
        prompt_path = Path(__file__).parent / "prompts" / "intent_parser_prompt.md"

        try:
            self.system_prompt = await self.llm_client.load_prompt_template(prompt_path)
            logger.info("Intent parser prompt loaded", path=str(prompt_path))
        except Exception as e:
            logger.error("Failed to load intent parser prompt", error=str(e))
            raise

    async def parse(self, command: VoiceCommand) -> ParsedIntent:
        """
        Parse voice command to structured intent.

        Args:
            command: Voice command to parse

        Returns:
            Parsed intent with action type and parameters

        Raises:
            ParsingError: If parsing fails
        """
        if not self.system_prompt:
            await self.initialize()

        logger.start("intent_parsing", command_id=str(command.id))
        start_time = asyncio.get_event_loop().time()

        try:
            # Call LLM with structured output
            parsed_data = await self.llm_client.structured_completion(
                system_prompt=self.system_prompt,
                user_message=command.transcribed_text,
                response_model=ParsedIntent,
                temperature=0.0,
            )

            # Set command_id
            parsed_data.command_id = command.id

            # Validate parameters for action type
            parsed_data.validate_parameters_for_action()

            duration_ms = (asyncio.get_event_loop().time() - start_time) * 1000
            logger.complete(
                "intent_parsing",
                duration_ms=duration_ms,
                action_type=parsed_data.action_type.value,
                requires_clarification=parsed_data.requires_clarification,
            )

            return parsed_data

        except Exception as e:
            logger.error("intent_parsing", e, command_text=command.transcribed_text)
            raise ParsingError(
                f"Intent parsing failed: {e}",
                llm_response="",
                command_text=command.transcribed_text,
            )


# Example usage
if __name__ == "__main__":
    import sys
    from uuid import uuid4

    async def test_intent_parser():
        """Test intent parser."""
        parser = IntentParser()
        await parser.initialize()

        # Test commands
        test_commands = [
            "Pick up the red block",
            "Move forward 2 meters",
            "Put the cup on the table",
            "Stop",
        ]

        for text in test_commands:
            command = VoiceCommand(
                id=uuid4(),
                audio_buffer=b"test",
                transcribed_text=text,
                confidence=0.95,
            )

            print(f"\nCommand: '{text}'")
            try:
                intent = await parser.parse(command)
                print(f"  Action: {intent.action_type.value}")
                print(f"  Objects: {len(intent.target_objects)}")
                print(f"  Parameters: {list(intent.parameters.keys())}")
                print(f"  Clarification needed: {intent.requires_clarification}")
            except ParsingError as e:
                print(f"  Error: {e}")

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_intent_parser())