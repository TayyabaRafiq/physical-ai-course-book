"""Tests for plan validator."""

import pytest
from uuid import uuid4

from src.vla_core.cognition.plan_validator import PlanValidator
from src.vla_core.models import ActionType
from src.vla_core.models.action_plan import ActionPlan
from src.vla_core.models.robot_action import RobotAction


@pytest.mark.asyncio
class TestPlanValidator:
    """Test PlanValidator."""

    async def test_validate_valid_plan(self):
        """Test validating a valid plan."""
        validator = PlanValidator()

        plan = ActionPlan(
            intent_id=uuid4(),
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
                    goal_message={"object_id": "block_1", "grasp_type": "top"},
                    timeout=5.0,
                ),
            ],
            preconditions=["Robot ready"],
            expected_outcomes=["Object picked"],
        )

        is_valid, errors = await validator.validate(plan)

        assert is_valid is True
        assert len(errors) == 0
        assert plan.validated is True

    async def test_validate_empty_plan(self):
        """Test validating empty plan."""
        validator = PlanValidator()

        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[],
            preconditions=[],
            expected_outcomes=[],
        )

        is_valid, errors = await validator.validate(plan)

        assert is_valid is False
        assert "no steps" in errors[0].lower()

    async def test_validate_missing_preconditions(self):
        """Test plan missing preconditions."""
        validator = PlanValidator()

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
            preconditions=[],  # Missing
            expected_outcomes=["Done"],
        )

        is_valid, errors = await validator.validate(plan)

        assert is_valid is False
        assert any("preconditions" in e.lower() for e in errors)

    async def test_validate_topological_error(self):
        """Test detecting topological errors."""
        validator = PlanValidator()

        # PLACE before PICK - should fail
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.PLACE,  # Wrong order!
                    ros_action_name="/place_object",
                    goal_message={"placement_pose": {"x": 1.0, "y": 1.0, "z": 0.5}},
                    timeout=5.0,
                ),
            ],
            preconditions=["Ready"],
            expected_outcomes=["Placed"],
        )

        is_valid, errors = await validator.validate(plan)

        assert is_valid is False
        assert any("topological" in e.lower() for e in errors)

    async def test_validate_excessive_force(self, mock_config):
        """Test detecting excessive force."""
        validator = PlanValidator()

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

        # Set excessive force
        plan.steps[0].constraints.max_force = 200.0

        is_valid, errors = await validator.validate(plan)

        assert is_valid is False
        assert any("force" in e.lower() for e in errors)

    async def test_validate_navigate_step(self):
        """Test validating navigate step."""
        validator = PlanValidator()

        # Valid navigation
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.NAVIGATE,
                    ros_action_name="/navigate",
                    goal_message={"target_pose": {"x": 2.0, "y": 1.0}},
                    timeout=10.0,
                )
            ],
            preconditions=["Ready"],
            expected_outcomes=["Done"],
        )

        is_valid, errors = await validator.validate(plan)
        assert is_valid is True

        # Invalid - too far
        plan.steps[0].goal_message["target_pose"] = {"x": 100.0, "y": 100.0}

        is_valid, errors = await validator.validate(plan)
        assert is_valid is False
        assert any("too far" in e.lower() for e in errors)

    async def test_validate_pick_step(self):
        """Test validating pick step."""
        validator = PlanValidator()

        # Missing object_id
        plan = ActionPlan(
            intent_id=uuid4(),
            steps=[
                RobotAction(
                    action_type=ActionType.PICK,
                    ros_action_name="/pick",
                    goal_message={},  # Missing object_id
                    timeout=5.0,
                )
            ],
            preconditions=["Ready"],
            expected_outcomes=["Done"],
        )

        is_valid, errors = await validator.validate(plan)
        assert is_valid is False
        assert any("object_id" in e.lower() for e in errors)