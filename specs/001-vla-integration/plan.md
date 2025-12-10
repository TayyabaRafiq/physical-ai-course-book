# Implementation Plan: Vision-Language-Action (VLA) Integration

**Branch**: `001-vla-integration` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-vla-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature implements a multi-modal voice-to-action pipeline for controlling a simulated humanoid robot. The system integrates speech recognition (Whisper), cognitive planning (LLM-based agentic reasoning), and robot control (ROS 2) to translate natural language commands into executable robot actions. The architecture follows a three-layer design: (1) Voice Interface Layer for audio capture and transcription, (2) Cognitive Planning Layer for intent parsing and action sequence generation, and (3) Robot Execution Layer for ROS 2 action dispatch and simulation control.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: OpenAI Whisper, LangChain/LangGraph (agentic framework), rclpy (ROS 2 Python client), PyBullet or Gazebo (simulation)
**Storage**: JSON-based execution logs, in-memory state management
**Testing**: pytest with ROS 2 integration test fixtures
**Target Platform**: Linux (Ubuntu 22.04) with ROS 2 Humble
**Project Type**: Single project with modular pipeline architecture
**Performance Goals**: <3 seconds voice-to-action latency for simple commands, <10 seconds end-to-end execution for single-step tasks
**Constraints**: Real-time audio processing, LLM API latency budget (1-2 seconds), simulation physics stability
**Scale/Scope**: Support 10+ command types, 3-5 step multi-action plans, single robot instance

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Status**: Constitution file is template - using general best practices

**Initial Check (Pre-Phase 0)**:
- Modular architecture with clear separation of concerns (Voice, Cognition, Execution layers)
- Test-first approach with integration tests for end-to-end pipeline
- Observability through structured logging at each pipeline stage
- Simplicity: start with core pipeline, defer advanced features (multi-robot, visual perception)

**Post-Phase 1 Re-evaluation**:
✅ **Architecture Modularity**: Strict layer boundaries enforced via Python interfaces (IAudioCapture, IIntentParser, IRosInterface)
✅ **Testing Strategy**: Contract tests for ROS actions, integration tests for pipeline, unit tests per layer
✅ **Observability**: ExecutionLog entity provides complete audit trail; structured logging at each stage
✅ **Simplicity**: Single project structure, minimal dependencies, deferred non-essential features
✅ **Data Integrity**: Pydantic schemas enforce validation rules at entity boundaries
✅ **Error Handling**: Explicit error hierarchy (VlaError subclasses) with graceful degradation

**Conclusion**: Design adheres to best practices. Ready for task breakdown.

## Project Structure

### Documentation (this feature)

```text
specs/001-vla-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── vla_core/
│   ├── __init__.py
│   ├── models/              # Data models (VoiceCommand, ParsedIntent, ActionPlan, etc.)
│   ├── voice/               # Voice interface layer
│   │   ├── audio_capture.py
│   │   ├── transcription.py (Whisper integration)
│   │   └── voice_interface.py
│   ├── cognition/           # Cognitive planning layer
│   │   ├── intent_parser.py (LLM-based NLU)
│   │   ├── action_planner.py (Agentic planning logic)
│   │   ├── plan_validator.py (Safety checks)
│   │   └── llm_client.py
│   ├── execution/           # Robot execution layer
│   │   ├── ros_interface.py (ROS 2 action client)
│   │   ├── action_translator.py (Plan → ROS messages)
│   │   └── execution_monitor.py
│   ├── pipeline/            # Orchestration
│   │   ├── vla_pipeline.py (Main coordinator)
│   │   └── state_manager.py
│   └── utils/
│       ├── logging_config.py
│       └── config.py
│
├── simulation/              # Simulation environment setup
│   ├── robot_description/   # URDF/SDF models
│   ├── worlds/              # Simulation worlds
│   └── launch/              # ROS 2 launch files
│
└── cli/
    └── vla_cli.py           # Command-line interface

tests/
├── unit/
│   ├── test_voice/
│   ├── test_cognition/
│   └── test_execution/
├── integration/
│   ├── test_pipeline_e2e.py
│   ├── test_ros_integration.py
│   └── fixtures/
│       └── mock_simulation.py
└── contract/
    └── test_ros_action_contracts.py
```

**Structure Decision**: Single project structure selected. The VLA system is a cohesive pipeline with tightly coupled components (voice → cognition → execution). Monolithic structure simplifies inter-layer communication, shared state management, and end-to-end testing. Separate `simulation/` directory isolates ROS-specific configuration from core Python logic.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations to report - architecture follows modular design, test-driven approach, and simplicity principles.

## Phase 0: Research & Technology Selection

### Research Areas

1. **Whisper Integration Pattern**
   - Question: Local Whisper model vs. OpenAI API for speech recognition?
   - Implications: Local = no network dependency but higher compute; API = faster but requires connectivity

2. **LLM Agentic Framework**
   - Question: LangChain vs. LangGraph vs. custom ReAct loop for cognitive planning?
   - Implications: Framework choice affects extensibility, tool integration, and debugging

3. **ROS 2 Action Server Architecture**
   - Question: Which ROS 2 action types to use for robot control (MoveBase, FollowJointTrajectory, custom)?
   - Implications: Determines level of abstraction and simulation compatibility

4. **Simulation Platform**
   - Question: Gazebo Classic vs. Gazebo Ignition vs. PyBullet for humanoid simulation?
   - Implications: ROS 2 integration maturity, physics realism, setup complexity

5. **Safety Validation Strategy**
   - Question: Pre-execution validation vs. real-time monitoring for collision/constraint checks?
   - Implications: Latency vs. safety tradeoffs

### Research Outputs → research.md

Will document: selected technologies, decision rationale, integration patterns, and example code snippets for each layer.

## Phase 1: Design Artifacts

### Data Model (data-model.md)

**Core Entities**:
- `VoiceCommand`: audio buffer, transcribed text, timestamp, confidence
- `ParsedIntent`: action type enum, target objects, parameters dict, ambiguities list
- `ActionPlan`: plan ID, steps list, pre-conditions, expected outcomes
- `RobotAction`: action type, ROS message payload, constraints, timeout
- `ExecutionState`: plan ID, current step index, status enum, error log

**State Transitions**: IDLE → LISTENING → TRANSCRIBING → PLANNING → EXECUTING → COMPLETED/FAILED

### API Contracts (contracts/)

**Internal Python Interfaces** (not REST/GraphQL - this is a local pipeline):
- `VoiceInterface.listen() → VoiceCommand`
- `IntentParser.parse(text: str) → ParsedIntent`
- `ActionPlanner.generate_plan(intent: ParsedIntent) → ActionPlan`
- `RosInterface.execute(plan: ActionPlan) → ExecutionState`

**ROS 2 Action Definitions** (IDL contracts):
- Will define custom action types for high-level commands (e.g., `PickObject.action`, `NavigateToPoint.action`)

### Quickstart (quickstart.md)

Installation, ROS 2 workspace setup, simulation launch, running first voice command.

## Architectural Decisions

### Layer Separation Strategy

**Decision**: Implement strict layer boundaries with adapter pattern
- Voice layer returns domain objects (VoiceCommand), not raw audio
- Cognition layer is LLM-agnostic (swap GPT-4 ↔ Claude via config)
- Execution layer is simulation-agnostic (Gazebo ↔ PyBullet via ROS interface)

**Rationale**: Testability and flexibility for future hardware integration

### Asynchronous Pipeline Flow

**Decision**: Use Python asyncio for pipeline stages
- Audio capture runs in background thread
- LLM calls are async HTTP requests
- ROS action execution uses ROS 2 async action client

**Rationale**: Meets <3 second latency requirement by parallelizing I/O-bound operations

### Error Recovery Pattern

**Decision**: Retry with degradation
- Speech recognition: retry with "please repeat" prompt
- LLM planning: fallback to simpler action if complex plan fails validation
- ROS execution: pause and request operator guidance on failure

**Rationale**: Balances autonomy with safety per FR-012 (graceful degradation)

## Integration Points

### Voice → Cognition
- Interface: `VoiceCommand` object with transcribed text
- Contract: Text must be non-empty, confidence >0.7 threshold

### Cognition → Execution
- Interface: `ActionPlan` with validated ROS-compatible actions
- Contract: All actions must pass safety validator before handoff

### Execution → Simulation
- Interface: ROS 2 action client/server protocol
- Contract: Standard ROS 2 action lifecycle (goal → feedback → result)

## Testing Strategy

### Unit Tests
- Voice: Mock audio input, verify Whisper transcription accuracy
- Cognition: Mock LLM responses, verify plan generation logic
- Execution: Mock ROS actions, verify message translation

### Integration Tests
- End-to-end: Real Whisper + real LLM + mock ROS (no simulation)
- ROS integration: Mock planning + real ROS + real simulation

### Contract Tests
- Verify ROS action message schemas match simulation expectations
- Validate JSON logging format for execution traces

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| LLM API latency exceeds budget | High | Implement local LLM fallback (e.g., Llama 2) |
| Whisper transcription errors | Medium | Add confidence thresholding + retry logic |
| ROS simulation instability | Medium | Add health checks + auto-restart on crash |
| Complex plans fail validation | Low | Start with simple command set, expand iteratively |

## Next Steps (Phase 2)

After `/sp.plan` completes, run `/sp.tasks` to generate testable task breakdown from this plan.