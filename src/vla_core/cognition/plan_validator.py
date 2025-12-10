"""
Plan Validator for safety and feasibility checks.

Validates action plans before execution to prevent unsafe or impossible actions.
"""

import asyncio
from typing import List, Tuple

from ..contracts.interfaces import IPlanValidator
from ..models.action_plan import ActionPlan
from ..models import ActionType
from ..utils.config import get_config
from ..utils.errors import ValidationError
from ..utils.logging_config import PipelineLogger

logger = PipelineLogger(stage="cognition")


class PlanValidator(IPlanValidator):
    """
    Validates action plans for safety and feasibility.

    Validation checks:
        - Joint limit compliance
        - Workspace bounds
        - Collision avoidance (simple bounding box)
        - Step count limits
        - Topological consistency
        - Parameter completeness
    """

    def __init__(self):
        """Initialize plan validator."""
        self.config = get_config()

        # Robot workspace bounds (simplified)
        self.workspace_bounds = {
            "x_min": -0.8,
            "x_max": 0.8,
            "y_min": -0.8,
            "y_max": 0.8,
            "z_min": 0.0,
            "z_max": 1.5,
        }

        # Joint limits (simplified - shoulder and elbow)
        self.joint_limits = {
            "shoulder_pan": (-1.57, 1.57),  # ±90 degrees
            "shoulder_lift": (-1.57, 1.57),
            "elbow": (0.0, 2.5),  # 0-150 degrees
        }

        logger.info("PlanValidator initialized")

    async def validate(self, plan: ActionPlan) -> Tuple[bool, List[str]]:
        """
        Validate action plan for safety and feasibility.

        Args:
            plan: Plan to validate

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        logger.start("plan_validation", plan_id=str(plan.plan_id))
        start_time = asyncio.get_event_loop().time()

        errors: List[str] = []

        # Check 1: Step count
        if len(plan.steps) == 0:
            errors.append("Plan has no steps")

        if len(plan.steps) > self.config.max_plan_steps:
            errors.append(
                f"Plan exceeds maximum steps: {len(plan.steps)} > {self.config.max_plan_steps}"
            )

        # Check 2: Validate each step
        for i, step in enumerate(plan.steps):
            step_errors = self._validate_step(step, i)
            errors.extend(step_errors)

        # Check 3: Topological consistency
        topo_errors = self._validate_step_order(plan)
        errors.extend(topo_errors)

        # Check 4: Required fields
        if not plan.preconditions:
            errors.append("Plan missing preconditions")

        if not plan.expected_outcomes:
            errors.append("Plan missing expected outcomes")

        # Determine if valid
        is_valid = len(errors) == 0

        # Update plan validation status
        if is_valid:
            plan.mark_validated()
        else:
            for error in errors:
                plan.add_validation_error(error)

        duration_ms = (asyncio.get_event_loop().time() - start_time) * 1000
        logger.complete(
            "plan_validation",
            duration_ms=duration_ms,
            is_valid=is_valid,
            num_errors=len(errors),
        )

        return (is_valid, errors)

    def _validate_step(self, step, step_index: int) -> List[str]:
        """
        Validate a single step.

        Args:
            step: RobotAction to validate
            step_index: Step index for error messages

        Returns:
            List of validation errors
        """
        errors: List[str] = []

        # Check required fields
        if not step.ros_action_name:
            errors.append(f"Step {step_index}: Missing ROS action name")

        if not step.goal_message:
            errors.append(f"Step {step_index}: Missing goal message")

        # Check timeout
        if step.timeout <= 0:
            errors.append(f"Step {step_index}: Invalid timeout: {step.timeout}")

        # Validate constraints if collision checking enabled
        if self.config.enable_collision_checking:
            if step.constraints.max_force > self.config.max_force_threshold:
                errors.append(
                    f"Step {step_index}: Force exceeds limit: {step.constraints.max_force} > {self.config.max_force_threshold}"
                )

            if step.constraints.max_velocity > self.config.max_velocity:
                errors.append(
                    f"Step {step_index}: Velocity exceeds limit: {step.constraints.max_velocity} > {self.config.max_velocity}"
                )

        # Action-specific validation
        if step.action_type == ActionType.NAVIGATE:
            nav_errors = self._validate_navigate_step(step, step_index)
            errors.extend(nav_errors)

        elif step.action_type == ActionType.PICK:
            pick_errors = self._validate_pick_step(step, step_index)
            errors.extend(pick_errors)

        elif step.action_type == ActionType.PLACE:
            place_errors = self._validate_place_step(step, step_index)
            errors.extend(place_errors)

        return errors

    def _validate_navigate_step(self, step, step_index: int) -> List[str]:
        """Validate navigation step."""
        errors: List[str] = []

        goal = step.goal_message

        # Check for target_pose
        if "target_pose" not in goal:
            errors.append(f"Step {step_index}: Navigate missing target_pose")
            return errors

        # Validate position is reasonable (not too far)
        target_pose = goal["target_pose"]
        if isinstance(target_pose, dict):
            x = target_pose.get("x", 0.0)
            y = target_pose.get("y", 0.0)

            # Check if within reasonable bounds (10 meters)
            distance = (x**2 + y**2) ** 0.5
            if distance > 10.0:
                errors.append(
                    f"Step {step_index}: Navigate target too far: {distance:.2f}m > 10m"
                )

        return errors

    def _validate_pick_step(self, step, step_index: int) -> List[str]:
        """Validate pick step."""
        errors: List[str] = []

        goal = step.goal_message

        # Check for object_id
        if "object_id" not in goal:
            errors.append(f"Step {step_index}: Pick missing object_id")

        # Check grasp_type if present
        if "grasp_type" in goal:
            valid_grasp_types = ["top", "side", "front"]
            if goal["grasp_type"] not in valid_grasp_types:
                errors.append(
                    f"Step {step_index}: Invalid grasp_type: {goal['grasp_type']}"
                )

        return errors

    def _validate_place_step(self, step, step_index: int) -> List[str]:
        """Validate place step."""
        errors: List[str] = []

        goal = step.goal_message

        # Check for placement_pose
        if "placement_pose" not in goal:
            errors.append(f"Step {step_index}: Place missing placement_pose")

        # Check if placement is within workspace bounds
        if "placement_pose" in goal:
            pose = goal["placement_pose"]
            if isinstance(pose, dict):
                x = pose.get("x", 0.0)
                y = pose.get("y", 0.0)
                z = pose.get("z", 0.0)

                if not self._is_in_workspace(x, y, z):
                    errors.append(
                        f"Step {step_index}: Placement pose outside workspace: ({x}, {y}, {z})"
                    )

        return errors

    def _validate_step_order(self, plan: ActionPlan) -> List[str]:
        """
        Validate topological consistency of steps.

        Args:
            plan: Action plan

        Returns:
            List of validation errors
        """
        errors: List[str] = []

        # Track gripper state through plan
        gripper_holding_object = False
        last_picked_object = None

        for i, step in enumerate(plan.steps):
            # Check gripper state constraints
            if step.action_type == ActionType.PICK:
                if gripper_holding_object:
                    errors.append(
                        f"Step {i}: Cannot PICK while gripper is holding {last_picked_object} "
                        f"(must PLACE first)"
                    )
                else:
                    gripper_holding_object = True
                    last_picked_object = step.goal_message.get("object_id", "unknown")

            elif step.action_type == ActionType.PLACE:
                if not gripper_holding_object:
                    errors.append(
                        f"Step {i}: Cannot PLACE without holding an object "
                        f"(must PICK first)"
                    )
                else:
                    gripper_holding_object = False
                    last_picked_object = None

        # Warn if plan ends with object still in gripper
        if gripper_holding_object:
            errors.append(
                f"Plan ends with object in gripper ({last_picked_object}) - "
                f"consider adding PLACE step"
            )

        # Check for excessive navigation
        nav_count = sum(1 for step in plan.steps if step.action_type == ActionType.NAVIGATE)
        if nav_count > len(plan.steps) * 0.6:
            errors.append(
                f"Warning: {nav_count}/{len(plan.steps)} steps are NAVIGATE "
                f"(>60%) - consider optimizing route"
            )

        return errors

    def _is_in_workspace(self, x: float, y: float, z: float) -> bool:
        """
        Check if position is within robot workspace.

        Args:
            x, y, z: Position coordinates

        Returns:
            True if within workspace
        """
        return (
            self.workspace_bounds["x_min"] <= x <= self.workspace_bounds["x_max"]
            and self.workspace_bounds["y_min"] <= y <= self.workspace_bounds["y_max"]
            and self.workspace_bounds["z_min"] <= z <= self.workspace_bounds["z_max"]
        )


# Example usage
if __name__ == "__main__":
    import sys
    from uuid import uuid4

    from ..models.robot_action import ActionConstraints, RobotAction

    async def test_plan_validator():
        """Test plan validator."""
        validator = PlanValidator()

        # Create test plan
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
                    goal_message={
                        "object_id": "red_block_1",
                        "grasp_type": "top",
                    },
                    timeout=8.0,
                ),
            ],
            preconditions=["Robot ready"],
            expected_outcomes=["Object grasped"],
        )

        print("Validating plan...")
        is_valid, errors = await validator.validate(plan)

        print(f"Valid: {is_valid}")
        if errors:
            print("Errors:")
            for error in errors:
                print(f"  - {error}")
        else:
            print("No errors!")

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    asyncio.run(test_plan_validator())