# Cognitive Planning with LLMs

## Overview

The Cognitive Planning layer bridges natural language understanding and robot action execution. It uses Large Language Models (LLMs) to convert voice commands into structured action plans that the robot can execute.

**Key Responsibilities**:
- Parse natural language commands
- Generate multi-step action plans
- Validate action feasibility
- Handle ambiguity and context
- Provide execution feedback

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Voice Command Input                            │
│  "Pick up the red block and place it on the     │
│   table, then navigate to the kitchen"          │
└──────────────────┬──────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────┐
│         LLM Cognitive Planner                   │
│  ┌───────────────────────────────────────────┐  │
│  │ System Prompt:                            │  │
│  │ "You are a robot action planner..."       │  │
│  │                                            │  │
│  │ Available Actions:                        │  │
│  │ - NAVIGATE(x, y, theta)                   │  │
│  │ - PICK(object_id, grasp_type)             │  │
│  │ - PLACE(target_surface, placement_pose)   │  │
│  │ - INSPECT(target_id, duration)            │  │
│  │ - WAIT(duration)                           │  │
│  └───────────────────────────────────────────┘  │
└──────────────────┬──────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────┐
│         Structured Action Plan (JSON)           │
│  {                                              │
│    "steps": [                                   │
│      {                                          │
│        "action_type": "PICK",                   │
│        "goal_message": {                        │
│          "object_id": "red_block",              │
│          "grasp_type": "top"                    │
│        },                                       │
│        "timeout": 8.0                           │
│      },                                         │
│      {                                          │
│        "action_type": "PLACE",                  │
│        "goal_message": {                        │
│          "target_surface": "table",             │
│          "placement_pose": {"x": 0.5, "y": 0.3} │
│        },                                       │
│        "timeout": 10.0                          │
│      },                                         │
│      {                                          │
│        "action_type": "NAVIGATE",               │
│        "goal_message": {                        │
│          "target_pose": {"x": 5.0, "y": 3.0}    │
│        },                                       │
│        "timeout": 15.0                          │
│      }                                          │
│    ]                                            │
│  }                                              │
└─────────────────────────────────────────────────┘
```

## LLM Selection

### Supported LLM Providers

| Provider | Model | Strengths | API Required |
|----------|-------|-----------|--------------|
| **OpenAI** | GPT-4o, GPT-4 Turbo | Best reasoning, JSON mode | Yes (paid) |
| **Anthropic** | Claude 3.5 Sonnet | Excellent instruction following | Yes (paid) |
| **Ollama** | Llama 3, Mistral | Local execution, free | No (local) |
| **OpenRouter** | Multiple models | Unified API | Yes (paid) |

**Recommendation**: Use **GPT-4o** for production, **Ollama** for development.

## Installation

### Install LLM Client Libraries

```bash
# OpenAI
pip install openai

# Anthropic
pip install anthropic

# Ollama (local)
pip install ollama

# For local model serving
# Install Ollama from https://ollama.ai/
```

### Set Up API Keys

```bash
# .env file
OPENAI_API_KEY=sk-...
ANTHROPIC_API_KEY=sk-ant-...
```

## Cognitive Planner Implementation

### Core CognitivePlanner Class

```python
# src/vla_core/services/cognitive_planner.py
from openai import OpenAI
from anthropic import Anthropic
import json
import os
from typing import Optional
from dataclasses import dataclass

@dataclass
class ActionPlan:
    """Structured action plan."""
    steps: list[dict]
    total_estimated_time: float
    plan_id: str

class CognitivePlanner:
    """LLM-based cognitive planner for robot actions."""

    SYSTEM_PROMPT = """You are a robot action planning assistant. Your job is to convert natural language commands into structured JSON action plans.

Available Actions:
1. NAVIGATE - Move to coordinates
   Parameters: target_pose {x: float, y: float, theta: float (optional)}
   Example: {"action_type": "NAVIGATE", "goal_message": {"target_pose": {"x": 2.0, "y": 1.0}}, "timeout": 10.0}

2. PICK - Grasp an object
   Parameters: object_id (string), grasp_type ("top" | "side" | "front")
   Example: {"action_type": "PICK", "goal_message": {"object_id": "red_block", "grasp_type": "top"}, "timeout": 8.0}

3. PLACE - Place held object
   Parameters: target_surface (string), placement_pose {x: float, y: float}
   Example: {"action_type": "PLACE", "goal_message": {"target_surface": "table", "placement_pose": {"x": 0.5, "y": 0.3}}, "timeout": 10.0}

4. INSPECT - Observe target with camera
   Parameters: target_id (string), duration (float)
   Example: {"action_type": "INSPECT", "goal_message": {"target_id": "workspace", "duration": 3.0}, "timeout": 5.0}

5. WAIT - Pause for duration
   Parameters: duration (float)
   Example: {"action_type": "WAIT", "goal_message": {"duration": 2.0}, "timeout": 5.0}

6. STOP - Emergency stop
   Example: {"action_type": "STOP", "goal_message": {}, "timeout": 1.0}

Output Format:
{
  "steps": [
    {
      "action_type": "ACTION_NAME",
      "goal_message": { /* action-specific params */ },
      "timeout": <float>
    }
  ]
}

Rules:
- Always output valid JSON
- Use realistic timeouts (navigate: 2s per meter, pick: 8s, place: 10s)
- Break complex commands into sequential steps
- For ambiguous commands, make reasonable assumptions
- Coordinates are in meters relative to robot's map frame
"""

    def __init__(
        self,
        provider: str = "openai",
        model: str = "gpt-4o",
        api_key: Optional[str] = None
    ):
        """
        Initialize cognitive planner.

        Args:
            provider: LLM provider ("openai", "anthropic", "ollama")
            model: Model name
            api_key: API key (reads from env if not provided)
        """
        self.provider = provider
        self.model = model

        if provider == "openai":
            self.client = OpenAI(api_key=api_key or os.getenv("OPENAI_API_KEY"))
        elif provider == "anthropic":
            self.client = Anthropic(api_key=api_key or os.getenv("ANTHROPIC_API_KEY"))
        elif provider == "ollama":
            import ollama
            self.client = ollama
        else:
            raise ValueError(f"Unsupported provider: {provider}")

    def plan_actions(self, voice_command: str) -> ActionPlan:
        """
        Generate action plan from voice command.

        Args:
            voice_command: Natural language command

        Returns:
            ActionPlan with structured steps
        """
        # Generate plan using LLM
        response = self._call_llm(voice_command)

        # Parse JSON response
        try:
            plan_data = json.loads(response)
        except json.JSONDecodeError as e:
            # Fallback: extract JSON from markdown code blocks
            import re
            json_match = re.search(r'```json\n(.*?)\n```', response, re.DOTALL)
            if json_match:
                plan_data = json.loads(json_match.group(1))
            else:
                raise ValueError(f"Failed to parse LLM response as JSON: {e}")

        # Validate and create ActionPlan
        steps = plan_data.get("steps", [])
        total_time = sum(step.get("timeout", 0) for step in steps)

        return ActionPlan(
            steps=steps,
            total_estimated_time=total_time,
            plan_id=self._generate_plan_id()
        )

    def _call_llm(self, user_message: str) -> str:
        """Call LLM based on provider."""

        if self.provider == "openai":
            return self._call_openai(user_message)
        elif self.provider == "anthropic":
            return self._call_anthropic(user_message)
        elif self.provider == "ollama":
            return self._call_ollama(user_message)

    def _call_openai(self, user_message: str) -> str:
        """Call OpenAI API."""
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.SYSTEM_PROMPT},
                {"role": "user", "content": user_message}
            ],
            response_format={"type": "json_object"},  # JSON mode
            temperature=0.3,  # Low temperature for consistency
            max_tokens=1000
        )

        return response.choices[0].message.content

    def _call_anthropic(self, user_message: str) -> str:
        """Call Anthropic API."""
        response = self.client.messages.create(
            model=self.model,
            max_tokens=1000,
            system=self.SYSTEM_PROMPT,
            messages=[
                {"role": "user", "content": user_message}
            ],
            temperature=0.3
        )

        return response.content[0].text

    def _call_ollama(self, user_message: str) -> str:
        """Call Ollama (local LLM)."""
        response = self.client.chat(
            model=self.model,
            messages=[
                {"role": "system", "content": self.SYSTEM_PROMPT},
                {"role": "user", "content": user_message}
            ],
            format="json",  # Force JSON output
            options={"temperature": 0.3}
        )

        return response['message']['content']

    def _generate_plan_id(self) -> str:
        """Generate unique plan ID."""
        import uuid
        return f"plan_{uuid.uuid4().hex[:8]}"
```

## Usage Examples

### Example 1: Basic Planning

```python
from src.vla_core.services.cognitive_planner import CognitivePlanner

# Initialize planner
planner = CognitivePlanner(provider="openai", model="gpt-4o")

# Generate plan
voice_command = "Pick up the red block"
plan = planner.plan_actions(voice_command)

print(f"Plan ID: {plan.plan_id}")
print(f"Total steps: {len(plan.steps)}")
print(f"Estimated time: {plan.total_estimated_time}s")

for i, step in enumerate(plan.steps):
    print(f"\nStep {i+1}: {step['action_type']}")
    print(f"  Goal: {step['goal_message']}")
    print(f"  Timeout: {step['timeout']}s")
```

**Output**:
```
Plan ID: plan_a3f2c8b1
Total steps: 1
Estimated time: 8.0s

Step 1: PICK
  Goal: {'object_id': 'red_block', 'grasp_type': 'top'}
  Timeout: 8.0s
```

### Example 2: Multi-Step Plan

```python
planner = CognitivePlanner(provider="openai", model="gpt-4o")

command = "Pick up the red block, place it on the table, then go to the kitchen"
plan = planner.plan_actions(command)

for i, step in enumerate(plan.steps):
    print(f"{i+1}. {step['action_type']}: {step['goal_message']}")
```

**Output**:
```
1. PICK: {'object_id': 'red_block', 'grasp_type': 'top'}
2. PLACE: {'target_surface': 'table', 'placement_pose': {'x': 0.5, 'y': 0.3}}
3. NAVIGATE: {'target_pose': {'x': 5.0, 'y': 3.0}}
```

### Example 3: Local LLM with Ollama

```python
# First, start Ollama and pull model:
# $ ollama pull llama3
# $ ollama serve

planner = CognitivePlanner(provider="ollama", model="llama3")
plan = planner.plan_actions("Navigate to coordinates x=2, y=1")

print(json.dumps(plan.steps, indent=2))
```

## Advanced Features

### Context-Aware Planning

```python
class ContextAwarePlanner(CognitivePlanner):
    """Planner with world state awareness."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.world_state = {}

    def update_world_state(self, state: dict):
        """Update known world state (object positions, robot location, etc.)"""
        self.world_state.update(state)

    def plan_actions(self, voice_command: str) -> ActionPlan:
        """Generate plan with world state context."""

        # Augment user message with context
        context = f"Current world state: {json.dumps(self.world_state)}\n\n"
        augmented_command = context + voice_command

        return super().plan_actions(augmented_command)

# Usage
planner = ContextAwarePlanner(provider="openai", model="gpt-4o")
planner.update_world_state({
    "robot_position": {"x": 0.0, "y": 0.0},
    "known_objects": ["red_block", "blue_cube"],
    "surfaces": ["table", "shelf"]
})

plan = planner.plan_actions("Pick up the nearest object")
# LLM can now use world state to determine "nearest object"
```

### Plan Validation

```python
class ValidatedPlanner(CognitivePlanner):
    """Planner with action validation."""

    VALID_ACTIONS = {"NAVIGATE", "PICK", "PLACE", "INSPECT", "WAIT", "STOP"}

    def plan_actions(self, voice_command: str) -> ActionPlan:
        """Generate and validate plan."""
        plan = super().plan_actions(voice_command)

        # Validate each step
        for i, step in enumerate(plan.steps):
            self._validate_step(step, i)

        return plan

    def _validate_step(self, step: dict, index: int):
        """Validate single action step."""
        action_type = step.get("action_type")

        if action_type not in self.VALID_ACTIONS:
            raise ValueError(f"Invalid action type at step {index}: {action_type}")

        if "goal_message" not in step:
            raise ValueError(f"Missing goal_message at step {index}")

        if "timeout" not in step or step["timeout"] <= 0:
            raise ValueError(f"Invalid timeout at step {index}")

        # Action-specific validation
        if action_type == "NAVIGATE":
            self._validate_navigate(step["goal_message"])
        elif action_type == "PICK":
            self._validate_pick(step["goal_message"])
        # ... other validations

    def _validate_navigate(self, goal: dict):
        """Validate NAVIGATE parameters."""
        if "target_pose" not in goal:
            raise ValueError("NAVIGATE missing target_pose")

        pose = goal["target_pose"]
        if "x" not in pose or "y" not in pose:
            raise ValueError("target_pose missing x or y coordinate")

    def _validate_pick(self, goal: dict):
        """Validate PICK parameters."""
        if "object_id" not in goal:
            raise ValueError("PICK missing object_id")

        if "grasp_type" not in goal:
            goal["grasp_type"] = "top"  # Default

        if goal["grasp_type"] not in ["top", "side", "front"]:
            raise ValueError(f"Invalid grasp_type: {goal['grasp_type']}")
```

### Conversational Refinement

```python
class ConversationalPlanner(CognitivePlanner):
    """Planner with multi-turn conversation."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.conversation_history = []

    def plan_with_clarification(self, voice_command: str) -> ActionPlan:
        """Generate plan, asking for clarification if needed."""

        self.conversation_history.append({
            "role": "user",
            "content": voice_command
        })

        response = self._call_llm_with_history()

        # Check if LLM is asking for clarification
        if self._is_clarification_request(response):
            return None  # Signal that clarification needed

        # Parse plan
        plan_data = json.loads(response)
        return ActionPlan(
            steps=plan_data["steps"],
            total_estimated_time=sum(s.get("timeout", 0) for s in plan_data["steps"]),
            plan_id=self._generate_plan_id()
        )

    def provide_clarification(self, clarification: str) -> ActionPlan:
        """Provide clarification and regenerate plan."""
        self.conversation_history.append({
            "role": "user",
            "content": clarification
        })

        return self.plan_with_clarification("")  # Empty command, using history

    def _is_clarification_request(self, response: str) -> bool:
        """Check if response is asking for clarification."""
        clarification_markers = [
            "which",
            "unclear",
            "specify",
            "could you clarify",
            "do you mean"
        ]
        return any(marker in response.lower() for marker in clarification_markers)
```

## Performance Optimization

### Caching

```python
from functools import lru_cache
import hashlib

class CachedPlanner(CognitivePlanner):
    """Planner with response caching."""

    @lru_cache(maxsize=100)
    def plan_actions(self, voice_command: str) -> ActionPlan:
        """Cache plans for identical commands."""
        return super().plan_actions(voice_command)
```

### Streaming Responses

```python
class StreamingPlanner(CognitivePlanner):
    """Planner with streaming responses for faster feedback."""

    def plan_actions_streaming(self, voice_command: str, callback):
        """Stream plan as it's generated."""

        if self.provider == "openai":
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.SYSTEM_PROMPT},
                    {"role": "user", "content": voice_command}
                ],
                stream=True
            )

            full_response = ""
            for chunk in stream:
                delta = chunk.choices[0].delta.content or ""
                full_response += delta
                callback(delta)  # Stream to UI

            return self._parse_plan(full_response)
```

## Testing

### Unit Tests

```python
import unittest
from src.vla_core.services.cognitive_planner import CognitivePlanner

class TestCognitivePlanner(unittest.TestCase):
    def setUp(self):
        self.planner = CognitivePlanner(provider="openai", model="gpt-4o")

    def test_simple_pick_command(self):
        plan = self.planner.plan_actions("Pick up the red block")

        self.assertEqual(len(plan.steps), 1)
        self.assertEqual(plan.steps[0]["action_type"], "PICK")
        self.assertIn("object_id", plan.steps[0]["goal_message"])

    def test_multi_step_command(self):
        plan = self.planner.plan_actions(
            "Pick up the block, place it on the table, then navigate to x=2, y=1"
        )

        self.assertEqual(len(plan.steps), 3)
        self.assertEqual(plan.steps[0]["action_type"], "PICK")
        self.assertEqual(plan.steps[1]["action_type"], "PLACE")
        self.assertEqual(plan.steps[2]["action_type"], "NAVIGATE")

    def test_navigate_command(self):
        plan = self.planner.plan_actions("Go to coordinates x=5, y=3")

        self.assertEqual(plan.steps[0]["action_type"], "NAVIGATE")
        pose = plan.steps[0]["goal_message"]["target_pose"]
        self.assertAlmostEqual(pose["x"], 5.0)
        self.assertAlmostEqual(pose["y"], 3.0)
```

## Best Practices

1. **Use JSON Mode**: Enable structured output for reliable parsing
2. **Low Temperature**: Use 0.3 or lower for consistent plans
3. **Validate Plans**: Always validate before execution
4. **Handle Ambiguity**: Ask for clarification rather than guessing
5. **Provide Context**: Include world state when available
6. **Cache Responses**: Avoid redundant LLM calls
7. **Set Timeouts**: Protect against LLM API latency
8. **Fallback Plans**: Have default behaviors for parsing failures

## Common Issues

### Issue: LLM returns invalid JSON

**Solution**: Use JSON mode and provide examples
```python
response_format={"type": "json_object"}  # OpenAI
format="json"  # Ollama
```

### Issue: Plans are inconsistent

**Solution**: Lower temperature
```python
temperature=0.1  # More deterministic
```

### Issue: Slow response times

**Solution**: Use smaller models or local LLMs
```python
provider="ollama", model="llama3"  # Local, fast
```

## Next Steps

- **[Chapter 3: Pure Python Execution](./03-python-execution)**: Execute generated plans
- **[Chapter 4: End-to-End Pipeline](./04-end-to-end-pipeline)**: Complete voice-to-action system

## Additional Resources

- [OpenAI JSON Mode](https://platform.openai.com/docs/guides/text-generation/json-mode)
- [Anthropic Claude API](https://docs.anthropic.com/claude/reference/messages)
- [Ollama Documentation](https://github.com/ollama/ollama)
- [Prompt Engineering Guide](https://www.promptingguide.ai/)
