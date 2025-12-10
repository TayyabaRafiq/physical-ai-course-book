# Tasks: Vision-Language-Action (VLA) Integration

**Input**: Design documents from `/specs/001-vla-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested - tests omitted per guideline

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## 🎯 **IMPORTANT: Pure Python MVP Approach**

The MVP implementation uses a **Pure Python ActionService** for execution instead of ROS 2. This simplifies the architecture and removes external dependencies, making the system:
- ✅ Easy to install and test (no ROS 2 setup required)
- ✅ Platform-independent (works on Windows, macOS, Linux)
- ✅ Fully functional for demonstration and development
- ✅ Ready to extend with ROS 2 backend when needed for real robots

**ROS 2 Integration**: Marked as **(OPTIONAL - Future)** throughout this document. All ROS 2-related tasks (T005-T012, T040-T041, T050-T053) are optional and can be implemented later for real robot control.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root per plan.md
- Simulation files in `src/simulation/`
- Core VLA code in `src/vla_core/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and ROS 2 workspace setup

- [X] T001 Create project directory structure per plan.md (src/vla_core/, src/simulation/, tests/)
- [X] T002 Initialize Python 3.11+ project with requirements.txt (whisper, langgraph, langchain-openai, pyaudio, pydantic, pytest)
- [X] T003 [P] Configure linting (black, isort) and type checking (mypy) in pyproject.toml
- [X] T004 [P] Create .env.template file with environment variables (OPENAI_API_KEY, WHISPER_MODEL, ROS_DOMAIN_ID, LOG_LEVEL)
- [~] T005 **(OPTIONAL - Future)** Create ROS 2 workspace structure (vla_ws/src/) - ROS 2 integration is optional
- [~] T006 **(OPTIONAL - Future)** Create vla_interfaces package for custom ROS 2 action definitions
- [~] T007 **(OPTIONAL - Future)** [P] Define PickObject.action ROS 2 action interface
- [~] T008 **(OPTIONAL - Future)** [P] Define PlaceObject.action ROS 2 action interface
- [~] T009 **(OPTIONAL - Future)** [P] Define NavigateToPoint.action ROS 2 action interface
- [~] T010 **(OPTIONAL - Future)** [P] Define InspectObject.action ROS 2 action interface
- [~] T011 **(OPTIONAL - Future)** Create CMakeLists.txt and package.xml for vla_interfaces package
- [~] T012 **(OPTIONAL - Future)** Build vla_interfaces package with colcon to generate Python message types
- [X] T013 [P] Setup Gazebo Fortress simulation world in src/simulation/worlds/household_world.sdf
- [X] T014 [P] Create humanoid robot URDF/SDF description in src/simulation/robot_description/humanoid.urdf
- [X] T015 Create ROS 2 launch file for simulation in src/simulation/launch/humanoid_sim.launch.py
- [X] T016 [P] Setup logging configuration in src/vla_core/utils/logging_config.py (structured JSON logging)
- [X] T017 [P] Create configuration management in src/vla_core/utils/config.py (load from .env)

**Checkpoint**: Project structure, ROS workspace, and simulation environment ready

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core data models, interfaces, and error handling that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T018 [P] Create ActionType enum in src/vla_core/models/__init__.py (NAVIGATE, PICK, PLACE, INSPECT, MANIPULATE, WAIT, STOP, UNKNOWN)
- [X] T019 [P] Create ExecutionStatus enum in src/vla_core/models/__init__.py (IDLE, RUNNING, PAUSED, COMPLETED, FAILED, CANCELLED)
- [X] T020 [P] Define VoiceCommand Pydantic model in src/vla_core/models/voice_command.py
- [X] T021 [P] Define ParsedIntent Pydantic model in src/vla_core/models/parsed_intent.py with validation rules
- [X] T022 [P] Define ActionPlan Pydantic model in src/vla_core/models/action_plan.py with validation rules
- [X] T023 [P] Define RobotAction Pydantic model in src/vla_core/models/robot_action.py with constraints schema
- [X] T024 [P] Define ExecutionState Pydantic model in src/vla_core/models/execution_state.py with state transition validation
- [X] T025 [P] Define ExecutionLog Pydantic model in src/vla_core/models/execution_log.py
- [X] T026 [P] Create error hierarchy in src/vla_core/utils/errors.py (VlaError, TranscriptionError, ParsingError, PlanningError, ValidationError, ExecutionError)
- [X] T027 [P] Define Python interface contracts in src/vla_core/contracts/interfaces.py (IAudioCapture, ITranscriber, IVoiceInterface, IIntentParser, IActionPlanner, IPlanValidator, IRosInterface, IExecutionMonitor, IVlaPipeline)
- [X] T028 Create StateManager class in src/vla_core/pipeline/state_manager.py for in-memory state tracking across pipeline stages

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Voice Command to Robot Action (Priority: P1) 🎯 MVP

**Goal**: Demonstrate complete voice-to-action pipeline with single-step commands (e.g., "Move forward 2 meters", "Pick up the red block")

**Independent Test**: Speak "Pick up the red block" and observe simulated robot execute pick-and-place action with complete traceability in execution logs

### Voice Interface Layer (US1)

- [X] T029 [P] [US1] Implement AudioCapture class in src/vla_core/voice/audio_capture.py using PyAudio for microphone input
- [X] T030 [P] [US1] Implement WhisperTranscriber class in src/vla_core/voice/transcription.py with local whisper-medium model loading
- [X] T031 [US1] Implement VoiceInterface class in src/vla_core/voice/voice_interface.py coordinating audio capture and transcription (depends on T029, T030)
- [X] T032 [US1] Add confidence threshold validation (>0.7) and retry logic for low-confidence transcriptions in src/vla_core/voice/voice_interface.py

### Cognitive Planning Layer (US1)

- [X] T033 [US1] Create LLM system prompt template for intent parsing in src/vla_core/cognition/prompts/intent_parser_prompt.md (describe robot capabilities, action types, parameter formats)
- [X] T034 [US1] Create LLM system prompt template for action planning in src/vla_core/cognition/prompts/action_planner_prompt.md (describe planning strategies, safety constraints, multi-step decomposition)
- [X] T035 [P] [US1] Implement LLMClient wrapper in src/vla_core/cognition/llm_client.py for OpenAI API with async calls and timeout handling
- [X] T036 [US1] Implement IntentParser class in src/vla_core/cognition/intent_parser.py using LLM with structured output (ParsedIntent Pydantic model)
- [X] T037 [US1] Implement ActionPlanner class in src/vla_core/cognition/action_planner.py using LangGraph StateGraph for plan generation
- [X] T038 [US1] Implement PlanValidator class in src/vla_core/cognition/plan_validator.py with joint limit, workspace bounds, and safety checks
- [X] T039 [US1] Add clarification handling logic in src/vla_core/cognition/intent_parser.py for ambiguous commands

### Execution Layer (US1)

- [~] T040 **(OPTIONAL - Future)** [P] [US1] Create ROS 2 action server skeleton for PickObject (for real robot integration)
- [~] T041 **(OPTIONAL - Future)** [P] [US1] Create ROS 2 action server skeleton for NavigateToPoint (for real robot integration)
- [X] T042 [P] [US1] Implement ActionService class in src/vla_core/execution/action_service.py with pure Python action execution
- [X] T043 [US1] Action translation built into ActionService (converts ActionPlan steps to simulated execution)
- [X] T044 [US1] Implement ExecutionMonitor class in src/vla_core/execution/execution_monitor.py to track plan execution state and handle feedback
- [X] T045 [US1] Add execution logging to ExecutionMonitor in src/vla_core/execution/execution_monitor.py (write ExecutionLog JSON to logs/executions/)

### Pipeline Orchestration (US1)

- [X] T046 [US1] Implement VlaPipeline class in src/vla_core/pipeline/vla_pipeline.py coordinating voice → cognition → execution flow with asyncio
- [X] T047 [US1] Add error recovery logic to VlaPipeline (retry transcription, fallback planning, pause on execution failure)
- [X] T048 [US1] Create CLI entry point in src/cli/vla_cli.py for running the pipeline with command-line arguments
- [X] T049 [US1] Add progress feedback display in CLI (transcription status, planning phase, execution progress percentage)

### Simulation Integration (US1) - **OPTIONAL - Not required for MVP**

- [~] T050 **(OPTIONAL - Future)** [US1] Implement PickObject action server logic (Gazebo physics - optional extension)
- [~] T051 **(OPTIONAL - Future)** [US1] Implement NavigateToPoint action server logic (nav2-style - optional extension)
- [~] T052 **(OPTIONAL - Future)** [US1] Configure ros_gz bridge (optional ROS 2 + Gazebo integration)
- [~] T053 **(OPTIONAL - Future)** [US1] Add simple objects to household_world.sdf for pick-and-place testing

**Checkpoint**: At this point, User Story 1 should be fully functional - operator can speak simple commands and observe robot execution with logs

---

## Phase 4: User Story 2 - Multi-Step Task Execution (Priority: P2)

**Goal**: Enable complex multi-step commands (e.g., "Clean the room", "Set the table") with autonomous task decomposition

**Independent Test**: Issue "Clean the room" command and verify system generates 3-5 step plan, executes sequentially, and reports completion

### Enhanced Cognitive Planning (US2)

- [ ] T054 [US2] Extend LLM action planning prompt in src/vla_core/cognition/prompts/action_planner_prompt.md with multi-step task decomposition strategies (sequencing, pre-conditions, outcome validation)
- [ ] T055 [US2] Implement multi-step plan generation in src/vla_core/cognition/action_planner.py using LangGraph conditional edges for replanning
- [ ] T056 [US2] Add plan complexity validation in src/vla_core/cognition/plan_validator.py (max 10 steps, topological consistency checks)
- [ ] T057 [US2] Implement replanning logic in src/vla_core/cognition/action_planner.py for failed sub-tasks (generate alternative steps)

### Enhanced Execution Layer (US2)

- [ ] T058 [US2] Add sequential step execution with progress tracking in src/vla_core/execution/execution_monitor.py (update current_step_index, completed_steps)
- [ ] T059 [US2] Implement sub-task failure handling in src/vla_core/execution/execution_monitor.py (pause, request guidance, attempt alternative)
- [ ] T060 [US2] Add expected outcome verification after each step in src/vla_core/execution/execution_monitor.py (compare actual vs. expected state)
- [ ] T061 [US2] Create PlaceObject action server in src/simulation/action_servers/place_object_server.py for object placement with stability checks

### Pipeline Enhancements (US2)

- [ ] T062 [US2] Add multi-step plan visualization in CLI (display all steps before execution, show current step during execution)
- [ ] T063 [US2] Implement clarification dialogue for missing information in src/vla_core/pipeline/vla_pipeline.py (e.g., "Where should I place the objects?")
- [ ] T064 [US2] Add intermediate status reporting in src/vla_core/pipeline/vla_pipeline.py (report after each step completion)

### Simulation Extensions (US2)

- [ ] T065 [US2] Add multiple scattered objects to household_world.sdf for "clean the room" scenario
- [ ] T066 [US2] Create designated disposal/storage areas in household_world.sdf (trash bin, shelf)
- [ ] T067 [US2] Implement object detection/tracking in simulation for dynamic scene understanding

**Checkpoint**: User Story 2 complete - system handles complex multi-step tasks with autonomous decomposition and sequential execution

---

## Phase 5: User Story 3 - Real-Time Voice Interaction (Priority: P3)

**Goal**: Support interruption commands during execution (Pause, Continue, Cancel, Emergency Stop) for dynamic control

**Independent Test**: Start long task, speak "Pause" mid-execution, verify robot halts and maintains state, then speak "Continue" and verify resumption

### Voice Layer Enhancements (US3)

- [ ] T068 [US3] Implement continuous background listening in src/vla_core/voice/audio_capture.py (thread-based audio stream monitoring)
- [ ] T069 [US3] Add interruption command detection in src/vla_core/voice/voice_interface.py (fast-path for Stop/Pause/Cancel keywords)
- [ ] T070 [US3] Implement voice activity detection (VAD) in src/vla_core/voice/audio_capture.py to filter silence and reduce processing

### Execution Control (US3)

- [ ] T071 [US3] Implement pause/resume logic in src/vla_core/execution/execution_monitor.py (save execution state, halt current action, resume from saved state)
- [ ] T072 [US3] Implement cancellation logic in src/vla_core/execution/execution_monitor.py (abort current action, mark plan as CANCELLED)
- [ ] T073 [US3] Add emergency stop handler in src/vla_core/execution/ros_interface.py (immediate action preemption)
- [ ] T074 [US3] Implement ROS action preemption/cancellation in src/vla_core/execution/ros_interface.py (call cancel_goal_async)

### Pipeline Coordination (US3)

- [ ] T075 [US3] Add interruption handling to VlaPipeline in src/vla_core/pipeline/vla_pipeline.py (listen for commands during execution)
- [ ] T076 [US3] Implement command priority queue in src/vla_core/pipeline/vla_pipeline.py (emergency stop > pause > new command)
- [ ] T077 [US3] Add state persistence for pause/resume in src/vla_core/pipeline/state_manager.py (serialize ExecutionState to disk)

### Simulation Support (US3)

- [ ] T078 [US3] Implement action preemption handling in all action servers (pick_object_server.py, navigate_server.py, place_object_server.py)
- [ ] T079 [US3] Add safe halt behavior in action servers (gradually stop motion, hold gripper state)

**Checkpoint**: User Story 3 complete - operator has full real-time control during task execution

---

## Phase 6: Documentation - Docusaurus Content

**Purpose**: Create comprehensive documentation explaining the VLA sequence execution

- [ ] T080 [P] Initialize Docusaurus site in docs/ directory with VLA Integration documentation structure
- [ ] T081 [P] Create Architecture Overview page in docs/docs/architecture/overview.md explaining three-layer design (Voice, Cognition, Execution)
- [ ] T082 [P] Create Voice Interface documentation in docs/docs/architecture/voice-layer.md (Whisper integration, audio capture, transcription flow)
- [ ] T083 [P] Create Cognitive Planning documentation in docs/docs/architecture/cognition-layer.md (LLM prompts, LangGraph state machine, planning strategies)
- [ ] T084 [P] Create Execution Layer documentation in docs/docs/architecture/execution-layer.md (ROS 2 actions, simulation integration, safety validation)
- [ ] T085 [P] Create VLA Sequence Flow diagram in docs/docs/sequence-flow.md with step-by-step walkthrough (audio → text → intent → plan → validation → execution → log)
- [ ] T086 [P] Create User Guide in docs/docs/user-guide.md with example commands and expected behaviors
- [ ] T087 [P] Create Developer Guide in docs/docs/developer-guide.md with extension points (adding new action types, custom LLM prompts, simulation environments)
- [ ] T088 [P] Create API Reference in docs/docs/api/ documenting all Python interfaces and ROS 2 actions
- [ ] T089 [P] Create Troubleshooting Guide in docs/docs/troubleshooting.md (common issues, debugging tips, log analysis)
- [ ] T090 Create Tutorial: First Voice Command in docs/docs/tutorials/first-command.md with step-by-step walkthrough
- [ ] T091 Create Tutorial: Multi-Step Task in docs/docs/tutorials/multi-step-task.md demonstrating "Clean the room" scenario
- [ ] T092 [P] Add Mermaid diagrams to sequence-flow.md showing complete pipeline execution with timing annotations
- [ ] T093 [P] Configure Docusaurus navigation and sidebar in docs/docusaurus.config.js
- [ ] T094 Build and deploy Docusaurus site to GitHub Pages or local server

**Checkpoint**: Complete documentation explaining VLA architecture, usage, and extension

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

- [ ] T095 [P] Add comprehensive error messages with recovery suggestions in src/vla_core/utils/errors.py
- [ ] T096 [P] Implement health checks for all pipeline components (Whisper loaded, LLM API reachable, ROS actions available)
- [ ] T097 [P] Add performance monitoring and latency tracking (measure voice-to-action time, log to ExecutionLog)
- [ ] T098 [P] Optimize Whisper inference (GPU acceleration verification, model caching)
- [ ] T099 [P] Add LLM response caching for common commands to reduce API latency
- [ ] T100 [P] Implement graceful shutdown handling (save state, close ROS connections, stop audio capture)
- [ ] T101 Create README.md at project root with quick start instructions
- [ ] T102 [P] Add configuration validation on startup (check required env vars, verify file paths)
- [ ] T103 Run quickstart.md validation (follow installation steps, verify first command execution)
- [ ] T104 [P] Add CI/CD pipeline configuration (if applicable) for automated testing and deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T017) - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (T018-T028) - MVP functionality
- **User Story 2 (Phase 4)**: Depends on User Story 1 (T029-T053) - Extends US1 with multi-step planning
- **User Story 3 (Phase 5)**: Depends on User Story 1 (T029-T053) - Can run in parallel with US2 if desired
- **Documentation (Phase 6)**: Can start after Phase 1 - Can run in parallel with implementation
- **Polish (Phase 7)**: Depends on desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Core dependency - MUST complete first for MVP
- **User Story 2 (P2)**: Depends on US1 (extends planning and execution layers)
- **User Story 3 (P3)**: Depends on US1 (adds interruption handling) - Independent of US2, can run in parallel

### Within Each User Story

1. Data models (Foundational phase) before services
2. Voice layer → Cognition layer → Execution layer → Pipeline orchestration
3. LLM prompt templates before LLM-dependent implementation
4. ROS action definitions before action server implementation
5. Core implementation before integration

### Parallel Opportunities

**Setup Phase**:
- T003 (linting), T004 (.env), T013 (world), T014 (URDF), T016 (logging), T017 (config) can run in parallel
- T007, T008, T009, T010 (all ROS actions) can run in parallel

**Foundational Phase**:
- All Pydantic models (T018-T025) can run in parallel
- T026 (errors), T027 (interfaces) can run in parallel with models

**User Story 1**:
- T029 (audio), T030 (whisper) can run in parallel
- T033 (intent prompt), T034 (planner prompt), T035 (LLM client) can run in parallel
- T040 (pick server), T041 (nav server), T042 (ROS interface) can run in parallel

**Documentation**:
- All documentation tasks (T080-T094) can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all foundational models together:
Task T020: "Define VoiceCommand model in src/vla_core/models/voice_command.py"
Task T021: "Define ParsedIntent model in src/vla_core/models/parsed_intent.py"
Task T022: "Define ActionPlan model in src/vla_core/models/action_plan.py"

# Launch all voice layer components together:
Task T029: "Implement AudioCapture in src/vla_core/voice/audio_capture.py"
Task T030: "Implement WhisperTranscriber in src/vla_core/voice/transcription.py"

# Launch all prompt templates together:
Task T033: "Create intent parser prompt template"
Task T034: "Create action planner prompt template"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T017) - ~2-3 hours
2. Complete Phase 2: Foundational (T018-T028) - ~2-3 hours - CRITICAL
3. Complete Phase 3: User Story 1 (T029-T053) - ~8-12 hours
4. **STOP and VALIDATE**: Test with simple commands ("Move forward", "Pick up red block")
5. Deploy/demo MVP

**MVP Scope**: 25 core tasks + 17 setup/foundational tasks = 42 tasks total

### Incremental Delivery

1. Setup + Foundational (Phase 1-2) → Foundation ready
2. Add User Story 1 (Phase 3) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (Phase 4) → Test multi-step tasks → Deploy/Demo
4. Add User Story 3 (Phase 5) → Test interruptions → Deploy/Demo
5. Add Documentation (Phase 6) → Publish documentation site
6. Polish (Phase 7) → Production-ready system

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (Phases 1-2)
2. Once Foundational is done:
   - Developer A: User Story 1 (Phase 3)
   - Developer B: Documentation (Phase 6) - can start early
3. After US1 complete:
   - Developer A: User Story 2 (Phase 4)
   - Developer C: User Story 3 (Phase 5) - parallel with US2
4. Final: Team collaborates on Polish (Phase 7)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label (US1, US2, US3) maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **LLM Prompts** (T033, T034, T054): Critical for cognitive layer - must be carefully crafted with robot capabilities and safety constraints
- **Docusaurus**: Provides comprehensive documentation for understanding VLA sequence execution
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Summary

**Total Tasks**: 104 tasks
**User Story 1 (MVP)**: 25 tasks (T029-T053)
**User Story 2**: 14 tasks (T054-T067)
**User Story 3**: 12 tasks (T068-T079)
**Documentation**: 15 tasks (T080-T094)
**Setup/Foundation**: 28 tasks (T001-T028)
**Polish**: 10 tasks (T095-T104)

**Parallel Opportunities**: 35+ tasks marked [P] can run in parallel
**Suggested MVP**: Complete Phases 1-3 (Setup + Foundational + User Story 1) for working voice-to-action demo