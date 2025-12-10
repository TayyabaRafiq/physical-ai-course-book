---
id: 002
title: VLA Technical Plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: vla-integration
branch: 001-vla-integration
user: unknown
command: /sp.plan
labels: ["architecture", "planning", "vla", "ros2", "whisper", "langgraph"]
links:
  spec: specs/001-vla-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-vla-integration/plan.md
  - specs/001-vla-integration/research.md
  - specs/001-vla-integration/data-model.md
  - specs/001-vla-integration/quickstart.md
  - specs/001-vla-integration/contracts/README.md
  - specs/001-vla-integration/contracts/python_interfaces.py
  - specs/001-vla-integration/contracts/PickObject.action
  - specs/001-vla-integration/contracts/PlaceObject.action
  - specs/001-vla-integration/contracts/NavigateToPoint.action
  - specs/001-vla-integration/contracts/InspectObject.action
  - CLAUDE.md (updated with tech stack)
tests:
  - Constitution Check validation (PASS)
---

## Prompt

Generate the technical plan for Module 4, defining the architecture for the voice-to-action pipeline (Whisper/Python stack), the agentic logic for LLM-based cognitive task planning, and the final ROS 2 action interface for execution control.

## Response snapshot

Created comprehensive implementation plan for VLA Integration with complete Phase 0 (research) and Phase 1 (design) artifacts:

**Phase 0 - Research & Technology Selection**:
1. **Whisper Integration**: Selected local whisper-medium model for offline capability and privacy
2. **Agentic Framework**: Chose LangGraph for explicit state management and debugging
3. **ROS 2 Actions**: Designed custom action definitions + nav2 integration
4. **Simulation**: Selected Gazebo Ignition (Fortress) for best ROS 2 compatibility
5. **Safety Validation**: Hybrid approach (pre-execution checks + runtime monitoring)

**Phase 1 - Design Artifacts**:
1. **Data Model** (data-model.md): 6 core entities with validation rules
   - VoiceCommand, ParsedIntent, ActionPlan, RobotAction, ExecutionState, ExecutionLog
2. **Contracts**: Python interfaces (ABCs) and ROS 2 action definitions
   - IAudioCapture, ITranscriber, IIntentParser, IActionPlanner, IRosInterface
   - PickObject.action, PlaceObject.action, NavigateToPoint.action, InspectObject.action
3. **Quickstart Guide**: Complete setup instructions from Ubuntu to running first command

**Architecture**:
- Three-layer design: Voice Interface → Cognitive Planning → Robot Execution
- Asyncio-based pipeline for <3 second latency requirement
- Strict layer boundaries via adapter pattern
- Error recovery: retry with degradation strategy

**Technical Stack**:
- Python 3.11+, Whisper (medium), LangGraph, OpenAI GPT-4, ROS 2 Humble, Gazebo Fortress
- Testing: pytest with ROS 2 integration fixtures

**Project Structure**: Single monolithic project with modular pipeline architecture in `src/vla_core/`

## Outcome

- ✅ Impact: Complete technical plan with all design decisions documented and justified
- 🧪 Tests: Constitution Check re-validation (PASS - all principles satisfied)
- 📁 Files: plan.md (234 lines), research.md (300+ lines), data-model.md (400+ lines), quickstart.md (250+ lines), 5 contract files
- 🔁 Next prompts: `/sp.tasks` to generate testable task breakdown, `/sp.adr` for architectural decisions
- 🧠 Reflection: Successfully resolved all technology choices through research phase; design artifacts are implementation-ready with clear contracts and validation rules

## Evaluation notes (flywheel)

- Failure modes observed: None - all phases completed without blocking issues
- Graders run and results (PASS/FAIL): Constitution Check post-design validation - PASS (modularity, testing, observability, simplicity all verified)
- Prompt variant (if applicable): Standard /sp.plan workflow with research phase
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to validate that plan provides sufficient detail for atomic task generation