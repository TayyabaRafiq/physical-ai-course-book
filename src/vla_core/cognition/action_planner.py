"""
Action Planner using LangGraph for multi-step task planning.

Generates action plans from parsed intents with LLM-based reasoning.
"""

import asyncio
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from pydantic import BaseModel

from ..contracts.interfaces import IActionPlanner
from ..models import ActionType
from ..models.action_plan import ActionPlan
from ..models.parsed_intent import ParsedIntent
from ..models.robot_action import ActionConstraints, RobotAction
from ..utils.config import get_config
from ..utils.errors import PlanningError
from ..utils.logging_config import PipelineLogger

from .llm_client import LLMClient

logger = PipelineLogger(stage="cognition")


class PlanResponse(BaseModel):
    """LLM response for action plan."""

    steps: List[Dict[str, Any]]
    preconditions: List[str]
    expected_outcomes: List[str]
    estimated_duration: float
    planning_error: Optional[str] = None


class ActionPlanner(IActionPlanner):
    """
    Generates action plans from intents using LLM.

    Features:
        - Multi-step task decomposition
        - Safety constraint application
        - Duration estimation
        - Replanning on failure
    """

    def __init__(self, llm_client: Optional[LLMClient] = None):
        """
        Initialize action planner.

        Args:
            llm_client: LLM client instance (None = create default)
        """
        self.config = get_config()
        self.llm_client = llm_client or LLMClient()

        # Load system prompt
        self.system_prompt: Optional[str] = None

        logger.info("ActionPlanner initialized")

    async def initialize(self) -> None:
        """Load prompt templates."""
        prompt_path = Path(__file__).parent / "prompts" / "action_planner_prompt.md"

        try:
            self.system_prompt = await self.llm_client.load_prompt_template(prompt_path)
            logger.info("Action planner prompt loaded", path=str(prompt_path))
        except Exception as e:
            logger.error("Failed to load action planner prompt", error=str(e))
            raise

    async def generate_plan(self, intent: ParsedIntent) -> ActionPlan:
        """
        Generate action plan from parsed intent.

        Args:
            intent: Parsed intent

        Returns:
            Action plan with ordered steps

        Raises:
            PlanningError: If planning fails
        """
        if not self.system_prompt:
            await self.initialize()

        logger.start("plan_generation", intent_id=str(intent.command_id))
        start_time = asyncio.get_event_loop().time()

        try:
            # Format intent as user message
            user_message = self._format_intent_message(intent)

            # Call LLM
            plan_response = await self.llm_client.structured_completion(
                system_prompt=self.system_prompt,
                user_message=user_message,
                response_model=PlanResponse,
                temperature=0.1,  # Slight randomness for planning
            )

            # Check for planning error
            if plan_response.planning_error:
                raise PlanningError(
                    plan_response.planning_error,
                    intent_id=str(intent.command_id),
                )

            # Convert to ActionPlan
            plan = self._convert_to_action_plan(intent, plan_response)

            # Validate step count
            if len(plan.steps) > self.config.max_plan_steps:
                raise PlanningError(
                    f"Plan exceeds maximum steps: {len(plan.steps)} > {self.config.max_plan_steps}",
                    plan_id=str(plan.plan_id),
                )

            duration_ms = (asyncio.get_event_loop().time() - start_time) * 1000
            logger.complete(
                "plan_generation",
                duration_ms=duration_ms,
                num_steps=len(plan.steps),
                estimated_duration=plan.estimated_duration,
            )

            return plan

        except PlanningError:
            raise
        except Exception as e:
            logger.error("plan_generation", e, intent_id=str(intent.command_id))
            raise PlanningError(
                f"Plan generation failed: {e}",
                intent_id=str(intent.command_id),
            )

    async def replan(
        self, intent: ParsedIntent, failed_step: int, error: str
    ) -> ActionPlan:
        """
        Regenerate plan after failure.

        Args:
            intent: Original intent
            failed_step: Index of step that failed
            error: Error description

        Returns:
            Alternative action plan

        Raises:
            PlanningError: If replanning fails
        """
        logger.info("Replanning after failure", failed_step=failed_step, error=error)

        try:
            # Format user message with failure context
            user_message = self._format_intent_message(intent)
            user_message += (
                f"\n\nPrevious plan failed at step {failed_step} with error: {error}\n"
                f"Generate an alternative plan avoiding this failure."
            )

            # Call LLM
            plan_response = await self.llm_client.structured_completion(
                system_prompt=self.system_prompt,
                user_message=user_message,
                response_model=PlanResponse,
                temperature=0.3,  # Higher temperature for alternative solutions
            )

            # Convert to ActionPlan
            plan = self._convert_to_action_plan(intent, plan_response)

            logger.info("Replanning successful", num_steps=len(plan.steps))
            return plan

        except Exception as e:
            logger.error("Replanning failed", error=str(e))
            raise PlanningError(
                f"Replanning failed: {e}",
                intent_id=str(intent.command_id),
            )

    def _format_intent_message(self, intent: ParsedIntent) -> str:
        """
        Format intent as user message for LLM.

        Args:
            intent: Parsed intent

        Returns:
            Formatted message
        """
        message = f"Action Type: {intent.action_type.value}\n\n"

        if intent.target_objects:
            message += "Target Objects:\n"
            for obj in intent.target_objects:
                message += f"  - {obj.name} ({obj.type})\n"
            message += "\n"

        if intent.parameters:
            message += "Parameters:\n"
            for key, value in intent.parameters.items():
                message += f"  - {key}: {value}\n"
            message += "\n"

        message += f"Generate a detailed action plan to accomplish this goal."

        return message

    def _convert_to_action_plan(
        self, intent: ParsedIntent, plan_response: PlanResponse
    ) -> ActionPlan:
        """
        Convert LLM response to ActionPlan.

        Args:
            intent: Original intent
            plan_response: LLM plan response

        Returns:
            Action plan with RobotAction steps
        """
        # Convert steps
        robot_actions: List[RobotAction] = []

        for step_data in plan_response.steps:
            # Extract fields
            action_type_str = step_data.get("action_type", "unknown")
            action_type = ActionType(action_type_str)

            ros_action_name = step_data.get("ros_action_name", "")
            goal_message = step_data.get("goal_message", {})
            timeout = step_data.get("timeout", 30.0)
            retry_on_failure = step_data.get("retry_on_failure", False)

            # Get or create constraints
            constraints_data = step_data.get("constraints", {})
            constraints = ActionConstraints(**constraints_data)

            # Create RobotAction
            action = RobotAction(
                action_type=action_type,
                ros_action_name=ros_action_name,
                goal_message=goal_message,
                constraints=constraints,
                timeout=timeout,
                retry_on_failure=retry_on_failure,
            )

            robot_actions.append(action)

        # Create ActionPlan
        plan = ActionPlan(
            intent_id=intent.command_id,
            steps=robot_actions,
            preconditions=plan_response.preconditions,
            expected_outcomes=plan_response.expected_outcomes,
            estimated_duration=plan_response.estimated_duration,
        )

        return plan


# Example usage
if __name__ == "__main__":
    import sys
    from uuid import uuid4

    async def test_action_planner():
        """Test action planner."""
        planner = ActionPlanner()
        await planner.initialize()

        # Create test intent
        intent = ParsedIntent(
            command_id=uuid4(),
            action_type=ActionType.PICK,
            target_objects=[],
            parameters={"object_id": "red_block_1", "grasp_type": "top"},
        )

        print("Generating plan...")
        try:
            plan = await planner.generate_plan(intent)
            print(f"  Steps: {len(plan.steps)}")
            print(f"  Duration: {plan.estimated_duration}s")
            print(f"  Preconditions: {plan.preconditions}")
            print(f"  Outcomes: {plan.expected_outcomes}")

            for i, step in enumerate(plan.steps):
                print(f"\n  Step {i + 1}: {step.action_type.value}")
                print(f"    ROS Action: {step.ros_action_name}")
                print(f"    Timeout: {step.timeout}s")

        except PlanningError as e:
            print(f"  Error: {e}")

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_action_planner())