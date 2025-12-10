"""
LLM Client wrapper for OpenAI API.

Provides async LLM calls with timeout handling and retry logic.
"""

import asyncio
import json
from pathlib import Path
from typing import Any, Dict, Optional

from openai import AsyncOpenAI
from pydantic import BaseModel

from ..utils.config import get_config
from ..utils.errors import ParsingError
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class LLMClient:
    """
    Wrapper for OpenAI API with structured output support.

    Features:
        - Async API calls
        - Timeout handling
        - Retry logic with exponential backoff
        - Structured output parsing with Pydantic
        - Response caching
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        timeout: Optional[float] = None,
    ):
        """
        Initialize LLM client.

        Args:
            api_key: OpenAI API key (None = use config)
            model: Model name (None = use config)
            timeout: Request timeout in seconds (None = use config)
        """
        config = get_config()

        self.api_key = api_key or config.openai_api_key
        self.model = model or config.openai_model
        self.timeout = timeout or config.llm_timeout

        self.client = AsyncOpenAI(api_key=self.api_key, timeout=self.timeout)

        self._cache: Dict[str, Any] = {}

        logger.info("LLMClient initialized", model=self.model, timeout=self.timeout)

    async def chat_completion(
        self,
        system_prompt: str,
        user_message: str,
        temperature: float = 0.0,
        max_tokens: Optional[int] = None,
        response_format: Optional[Dict] = None,
    ) -> str:
        """
        Call LLM with chat completion.

        Args:
            system_prompt: System prompt
            user_message: User message
            temperature: Sampling temperature (0.0 = deterministic)
            max_tokens: Maximum response tokens
            response_format: Response format spec (e.g., {"type": "json_object"})

        Returns:
            LLM response text

        Raises:
            ParsingError: If API call fails
        """
        # Check cache
        cache_key = f"{system_prompt}:{user_message}:{temperature}"
        if cache_key in self._cache:
            logger.debug("Using cached LLM response")
            return self._cache[cache_key]

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message},
        ]

        try:
            logger.debug(
                "Calling LLM",
                model=self.model,
                temperature=temperature,
                user_message_length=len(user_message),
            )

            # Make API call
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
                response_format=response_format,
            )

            # Extract response text
            response_text = response.choices[0].message.content

            if not response_text:
                raise ParsingError("Empty LLM response")

            # Cache response
            self._cache[cache_key] = response_text

            logger.debug(
                "LLM response received",
                response_length=len(response_text),
                tokens_used=response.usage.total_tokens if response.usage else None,
            )

            return response_text

        except Exception as e:
            logger.error("LLM API call failed", error=str(e))
            raise ParsingError(f"LLM API call failed: {e}", llm_response="")

    async def structured_completion(
        self,
        system_prompt: str,
        user_message: str,
        response_model: type[BaseModel],
        temperature: float = 0.0,
        max_retries: int = 3,
    ) -> BaseModel:
        """
        Call LLM and parse response into structured Pydantic model.

        Args:
            system_prompt: System prompt
            user_message: User message
            response_model: Pydantic model to parse response into
            temperature: Sampling temperature
            max_retries: Maximum retry attempts on parse failure

        Returns:
            Parsed Pydantic model instance

        Raises:
            ParsingError: If parsing fails after retries
        """
        for attempt in range(max_retries):
            try:
                # Get raw response
                response_text = await self.chat_completion(
                    system_prompt=system_prompt,
                    user_message=user_message,
                    temperature=temperature,
                    response_format={"type": "json_object"},
                )

                # Parse JSON
                response_json = json.loads(response_text)

                # Validate with Pydantic
                parsed = response_model(**response_json)

                logger.debug("Structured completion successful", model=response_model.__name__)
                return parsed

            except json.JSONDecodeError as e:
                logger.warning(
                    "JSON parse failed",
                    attempt=attempt + 1,
                    error=str(e),
                    response_text=response_text[:200],
                )

                if attempt >= max_retries - 1:
                    raise ParsingError(
                        f"Failed to parse JSON after {max_retries} attempts",
                        llm_response=response_text,
                    )

                # Retry with higher temperature
                temperature += 0.1
                await asyncio.sleep(1.0)

            except Exception as e:
                logger.error(
                    "Structured parsing failed",
                    attempt=attempt + 1,
                    error=str(e),
                )

                if attempt >= max_retries - 1:
                    raise ParsingError(
                        f"Failed to parse structured output: {e}",
                        llm_response=response_text if "response_text" in locals() else "",
                    )

                await asyncio.sleep(1.0)

        # Should not reach here
        raise ParsingError("Structured completion failed (unexpected)")

    async def load_prompt_template(self, template_path: Path) -> str:
        """
        Load prompt template from file.

        Args:
            template_path: Path to prompt template

        Returns:
            Template content
        """
        try:
            with open(template_path, "r", encoding="utf-8") as f:
                return f.read()
        except Exception as e:
            logger.error("Failed to load prompt template", path=str(template_path), error=str(e))
            raise ParsingError(f"Failed to load prompt template: {e}")

    def clear_cache(self) -> None:
        """Clear response cache."""
        self._cache.clear()
        logger.debug("LLM cache cleared")


# Example usage
if __name__ == "__main__":
    import sys
    from pydantic import Field

    class TestResponse(BaseModel):
        """Test response model."""

        action: str
        confidence: float = Field(ge=0.0, le=1.0)

    async def test_llm_client():
        """Test LLM client."""
        client = LLMClient()

        # Simple completion
        response = await client.chat_completion(
            system_prompt="You are a helpful assistant.",
            user_message="Say hello!",
        )
        print(f"Response: {response}")

        # Structured completion
        structured = await client.structured_completion(
            system_prompt="Extract the action and confidence from the user message. Respond with JSON: {\"action\": \"...\", \"confidence\": 0.95}",
            user_message="Pick up the red block",
            response_model=TestResponse,
        )
        print(f"Structured: action={structured.action}, confidence={structured.confidence}")

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_llm_client())