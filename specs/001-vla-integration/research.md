# Research & Technology Decisions: VLA Integration

**Date**: 2025-12-06
**Feature**: Vision-Language-Action (VLA) Integration
**Purpose**: Document technology selection, integration patterns, and architectural decisions for the voice-to-action pipeline

## Research Area 1: Whisper Integration Pattern

### Decision
Use **local Whisper model** (whisper-medium) via OpenAI's open-source library

### Rationale
- **Offline capability**: No network dependency eliminates API latency (200-500ms) and cost
- **Privacy**: Audio data stays local - critical for future physical robot deployments
- **Latency**: Local inference on GPU (~1-2 seconds) meets <3 second budget
- **Cost**: Zero marginal cost vs. $0.006/minute for Whisper API
- **Trade-off accepted**: Higher initial setup (model download ~1.5GB) and GPU requirement

### Alternatives Considered
- **OpenAI Whisper API**: Rejected due to network dependency and recurring cost
- **Google Speech-to-Text**: Rejected - requires GCP setup, less accurate for robotics commands
- **Vosk**: Rejected - lower accuracy than Whisper for out-of-vocabulary technical terms

### Integration Pattern
```python
import whisper

class WhisperTranscriber:
    def __init__(self):
        self.model = whisper.load_model("medium")  # Balance speed/accuracy

    def transcribe(self, audio_array: np.ndarray) -> tuple[str, float]:
        result = self.model.transcribe(audio_array, language="en")
        return result["text"], result.get("confidence", 1.0)
```

### Best Practices
- Use `whisper-medium` for balance (base too inaccurate, large too slow)
- Enable VAD (voice activity detection) to reduce processing of silence
- Cache model in memory (don't reload per request)
- Run inference on GPU if available (10x faster than CPU)

---

## Research Area 2: LLM Agentic Framework

### Decision
Use **LangGraph** for cognitive planning with structured state management

### Rationale
- **Explicit state graphs**: LangGraph's StateGraph matches our IDLE→LISTENING→PLANNING→EXECUTING flow
- **Tool integration**: Built-in support for function calling (robot capability queries)
- **Debugging**: Visual graph inspection and step-by-step execution traces
- **Extensibility**: Easy to add clarification loops, replanning, and error recovery nodes
- **Production-ready**: Better error handling than custom ReAct loops

### Alternatives Considered
- **LangChain LCEL**: Rejected - linear chains don't support conditional branching (needed for clarification)
- **Custom ReAct loop**: Rejected - reinventing state management and error handling
- **AutoGen**: Rejected - overkill for single-agent system, adds complexity

### Integration Pattern
```python
from langgraph.graph import StateGraph, END
from langchain_openai import ChatOpenAI

class CognitivePlanner:
    def __init__(self):
        self.llm = ChatOpenAI(model="gpt-4", temperature=0)
        self.graph = self._build_graph()

    def _build_graph(self):
        workflow = StateGraph(PlanningState)
        workflow.add_node("parse_intent", self.parse_intent)
        workflow.add_node("generate_plan", self.generate_plan)
        workflow.add_node("validate_plan", self.validate_plan)
        workflow.add_node("request_clarification", self.request_clarification)

        workflow.add_edge("parse_intent", "generate_plan")
        workflow.add_conditional_edges(
            "generate_plan",
            self.should_validate,
            {"validate": "validate_plan", "clarify": "request_clarification"}
        )
        workflow.add_edge("validate_plan", END)

        return workflow.compile()
```

### Best Practices
- Use structured output with Pydantic models (not free-text JSON)
- Implement timeout on LLM calls (2 second budget)
- Cache robot capability descriptions in system prompt
- Use streaming for real-time feedback during planning

---

## Research Area 3: ROS 2 Action Server Architecture

### Decision
Use **custom ROS 2 action definitions** for high-level commands + standard `nav2` actions for navigation

### Rationale
- **Abstraction level**: High-level actions (PickObject, PlaceObject) hide low-level trajectory details
- **Simulation agnostic**: Custom actions work across Gazebo, PyBullet via adapter pattern
- **Feedback mechanism**: ROS 2 actions provide progress updates for multi-step tasks
- **Standard compatibility**: Reuse nav2_msgs/NavigateToPose for navigation (don't reinvent)

### Alternatives Considered
- **Low-level joint control**: Rejected - too detailed for voice commands, planning complexity explodes
- **ROS 2 services**: Rejected - no built-in feedback/cancellation mechanism
- **Pure topics**: Rejected - no request/response semantics, hard to track task completion

### Integration Pattern

**Custom Action Definition** (`PickObject.action`):
```
# Goal
string object_id
geometry_msgs/Point approach_offset
---
# Result
bool success
string error_message
geometry_msgs/Pose final_grasp_pose
---
# Feedback
string current_phase  # "approaching" | "grasping" | "lifting"
float32 progress_percent
```

**Python Action Client**:
```python
import rclpy
from rclpy.action import ActionClient
from vla_interfaces.action import PickObject

class RosActionExecutor:
    def __init__(self, node):
        self.pick_client = ActionClient(node, PickObject, 'pick_object')

    async def execute_pick(self, object_id: str) -> ExecutionResult:
        goal = PickObject.Goal(object_id=object_id)
        future = await self.pick_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        result = await future.get_result_async()
        return ExecutionResult(success=result.success, error=result.error_message)
```

### Best Practices
- Define one action per atomic task (Pick, Place, Navigate, Inspect)
- Include progress feedback for tasks >2 seconds
- Use `geometry_msgs` for poses (don't invent custom types)
- Implement preemption for task cancellation

---

## Research Area 4: Simulation Platform

### Decision
Use **Gazebo Ignition (Fortress)** with ROS 2 Humble integration

### Rationale
- **ROS 2 native**: Best integration via ros_gz bridge (topics, actions, services)
- **Physics realism**: ODE/Bullet physics for stable manipulation and contact dynamics
- **Humanoid support**: Pre-built models available (e.g., PAL Robotics TALOS, TIAGo)
- **Sensor simulation**: Built-in camera, IMU, force-torque sensors for future extensions
- **Community**: Active development, well-documented for robotics research

### Alternatives Considered
- **PyBullet**: Rejected - weaker ROS 2 integration, requires custom bridges
- **Gazebo Classic**: Rejected - deprecated, ROS 2 support is retrofit
- **Isaac Sim**: Rejected - requires NVIDIA GPU, overkill for MVP, steep learning curve

### Integration Pattern

**Launch File** (`simulation.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription('gz_sim.launch.py', launch_arguments={
            'world': 'household_world.sdf',
            'robot': 'humanoid_description'
        }),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
        Node(
            package='vla_core',
            executable='action_server',
            name='pick_place_server'
        )
    ])
```

### Best Practices
- Use SDF format for worlds (more expressive than URDF)
- Bridge only necessary topics (minimize overhead)
- Run Gazebo headless for CI/automated testing
- Use `.world` files for reproducible test scenarios

---

## Research Area 5: Safety Validation Strategy

### Decision
**Hybrid approach**: Pre-execution collision checking + real-time constraint monitoring

### Rationale
- **Pre-execution**: Fast rejection of obviously unsafe plans (1-10ms overhead acceptable)
- **Runtime monitoring**: Catch dynamic obstacles or plan deviations during execution
- **Latency balance**: Pre-checks avoid starting doomed tasks; runtime handles unknowns
- **Safety requirement**: Meets FR-008 (collision avoidance, joint limits, stability)

### Pre-Execution Checks
1. **Joint limits**: Verify all target positions within `[q_min, q_max]`
2. **Workspace bounds**: Check target poses within robot reachable zone
3. **Static collision**: Use simple bounding box checks (not full mesh collision)
4. **Stability**: For humanoids, verify center of mass stays within support polygon

### Runtime Monitoring
1. **Force thresholds**: Abort if joint torques exceed safe limits
2. **Divergence detection**: Stop if actual trajectory deviates >5cm from planned
3. **Timeout**: Cancel actions exceeding expected duration by 2x

### Integration Pattern
```python
class PlanValidator:
    def __init__(self, robot_model):
        self.joint_limits = robot_model.get_joint_limits()
        self.workspace_bounds = robot_model.get_workspace()

    def validate_plan(self, plan: ActionPlan) -> ValidationResult:
        for action in plan.steps:
            if not self._check_joint_limits(action):
                return ValidationResult(valid=False, reason="Joint limit violation")
            if not self._check_workspace(action):
                return ValidationResult(valid=False, reason="Unreachable pose")
        return ValidationResult(valid=True)
```

### Best Practices
- Use URDF kinematic model for reachability checks
- Pre-compute workspace convex hull offline (fast point-in-polytope test)
- Add 10% safety margin to limits (account for model uncertainty)
- Log all validation failures for plan refinement

---

## Technology Stack Summary

| Layer | Technology | Version | Purpose |
|-------|------------|---------|---------|
| Voice | OpenAI Whisper | whisper-medium (local) | Speech-to-text transcription |
| Audio | PyAudio | 0.2.13 | Microphone capture |
| Cognition | LangGraph | 0.0.40+ | Agentic planning state machine |
| LLM | OpenAI GPT-4 | gpt-4-turbo | Intent parsing & plan generation |
| Execution | rclpy | ROS 2 Humble | ROS Python client library |
| Simulation | Gazebo Ignition | Fortress (6.x) | Physics-based robot simulation |
| Bridge | ros_gz | Humble | ROS 2 ↔ Gazebo message passing |
| Testing | pytest | 7.4+ | Unit & integration tests |
| Async | asyncio | Python 3.11 stdlib | Concurrent pipeline execution |

## Integration Flow Diagram

```
┌─────────────┐
│  Operator   │
└──────┬──────┘
       │ voice
       ▼
┌─────────────────────────────────────────┐
│  Voice Layer (Python)                   │
│  ┌──────────┐    ┌─────────────┐       │
│  │ PyAudio  │───▶│   Whisper   │       │
│  │ Capture  │    │ Transcriber │       │
│  └──────────┘    └──────┬──────┘       │
└────────────────────────┼────────────────┘
                         │ text
                         ▼
┌─────────────────────────────────────────┐
│  Cognition Layer (LangGraph)            │
│  ┌──────────┐    ┌──────────┐          │
│  │  Intent  │───▶│  Action  │          │
│  │  Parser  │    │ Planner  │          │
│  └──────────┘    └─────┬────┘          │
│       │  (GPT-4)       │                │
│       │                ▼                │
│       │         ┌──────────────┐        │
│       └────────▶│  Validator   │        │
│                 └──────┬───────┘        │
└────────────────────────┼────────────────┘
                         │ ActionPlan
                         ▼
┌─────────────────────────────────────────┐
│  Execution Layer (ROS 2)                │
│  ┌──────────────┐    ┌──────────────┐  │
│  │   Action     │───▶│     ROS      │  │
│  │  Translator  │    │ Action Client│  │
│  └──────────────┘    └──────┬───────┘  │
└────────────────────────────┼────────────┘
                             │ ROS msgs
                             ▼
┌─────────────────────────────────────────┐
│  Simulation (Gazebo Ignition)           │
│  ┌──────────────────────────────┐      │
│  │   Humanoid Robot Model       │      │
│  │  (Physics + Kinematics)      │      │
│  └──────────────────────────────┘      │
└─────────────────────────────────────────┘
```

## Next Steps

All technology selections are finalized. Proceed to Phase 1:
1. Generate `data-model.md` with entity schemas
2. Define ROS action interfaces in `contracts/`
3. Create `quickstart.md` for developer onboarding