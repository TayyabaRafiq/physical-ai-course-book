# Tasks: AI-Robot Brain: High-Performance Perception and Navigation

**Input**: Design documents from `/specs/001-isaac-robot-brain/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md

**Tests**: Tests are OPTIONAL and not explicitly requested in the feature specification for this phase.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure, including documentation setup.

- [ ] T001 Create base project structure for the robot control module in `src/robot_brain/`
- [ ] T002 Initialize Docusaurus documentation project at `docs/robot-brain/`
- [ ] T003 Create Docusaurus documentation directories: `docs/robot-brain/perception/`, `docs/robot-brain/navigation/`, `docs/robot-brain/simulation/`, `docs/robot-brain/tutorials/`
- [ ] T004 Configure Docusaurus sidebars to reflect the planned documentation structure in `docs/robot-brain/sidebars.js`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Set up NVIDIA Isaac Sim environment, ensuring all dependencies are met for ROS 2 bridge
- [ ] T006 Install ROS 2 (Humble/Jazzy) on the target platform
- [ ] T007 Configure ROS 2 Bridge Extension for Isaac Sim, enabling communication between simulation and ROS 2 nodes
- [ ] T008 Install Isaac ROS VSLAM and Isaac ROS nvblox packages
- [ ] T009 Install Nav2 stack and its dependencies

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Autonomous Bipedal Movement (Priority: P1) 🎯 MVP

**Goal**: A robot autonomously navigates its environment, avoiding obstacles and reaching a destination.

**Independent Test**: Can be fully tested by activating the robot in a test environment with a set destination, and observing if it reaches the goal without collisions.

### Implementation for User Story 1

- [ ] T010 [US1] Configure Isaac ROS VSLAM launch files for visual odometry (VO) in `src/robot_brain/launch/vslam_vo.launch.py`
- [ ] T011 [US1] Integrate VSLAM odometry output into Nav2's localization or odometry source (e.g., `odometry/filtered` topic)
- [ ] T012 [US1] Configure Nav2 global and local planners for bipedal movement characteristics in `src/robot_brain/config/nav2_params.yaml`
- [ ] T013 [US1] Develop ROS 2 node to translate Nav2 movement commands into bipedal robot control signals for Isaac Sim in `src/robot_brain/nodes/bipedal_controller.py`
- [ ] T014 [US1] Implement obstacle detection and avoidance logic within the Nav2 stack or a custom layer for dynamic obstacles in `src/robot_brain/plugins/dynamic_obstacle_layer.cpp`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Real-time Environmental Understanding (Priority: P1)

**Goal**: The robot perceives and understands its environment in real-time using high-performance processing to inform navigation decisions.

**Independent Test**: Can be fully tested by placing the robot in an environment with various objects and verifying that its perception system generates an accurate, real-time map.

### Implementation for User Story 2

- [ ] T015 [US2] Configure Isaac ROS nvblox for real-time 3D reconstruction from simulated depth camera data in `src/robot_brain/launch/nvblox.launch.py`
- [ ] T016 [US2] Integrate nvblox-generated costmap output into Nav2's costmap layers for path planning in `src/robot_brain/config/nav2_costmap_params.yaml`
- [ ] T017 [US2] Develop ROS 2 node for real-time visualization of the robot's perception data (point clouds, occupancy map) in RViz in `src/robot_brain/nodes/perception_visualizer.py`
- [ ] T018 [US2] Implement continuous and accurate updates of the environmental representation based on new sensor data in `src/robot_brain/services/map_updater.py`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Visual Localization and Mapping (Priority: P2)

**Goal**: The robot uses visual cues to accurately determine its position and build a map of unknown environments simultaneously.

**Independent Test**: Can be fully tested by allowing the robot to explore an unknown area and then verifying the consistency and accuracy of the generated map and its self-localization within it.

### Implementation for User Story 3

- [ ] T019 [US3] Configure Isaac ROS VSLAM for full SLAM mode (including loop closure) in `src/robot_brain/launch/vslam_slam.launch.py`
- [ ] T020 [US3] Develop ROS 2 node to integrate SLAM-generated 3D map with Nav2's map server for navigation in `src/robot_brain/nodes/map_integrator.py`
- [ ] T021 [US3] Implement re-localization mechanism within Nav2 using VSLAM's pose estimates for previously mapped areas in `src/robot_brain/config/nav2_relocalization.yaml`

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Robot Simulation and Testing (Priority: P2)

**Goal**: Developers simulate the robot's behavior and test perception/navigation algorithms in a realistic virtual environment.

**Independent Test**: Can be fully tested by running the robot's perception and navigation stack within a simulation environment and verifying data flow and robot movement.

### Implementation for User Story 4

- [ ] T022 [US4] Import/configure bipedal robot model and relevant sensors in Isaac Sim environment (`isaac_sim_assets/bipedal_robot.usd`)
- [ ] T023 [US4] Configure Isaac Sim to stream simulated sensor data (camera, lidar, IMU) to ROS 2 topics via the ROS 2 bridge in `isaac_sim_configs/sensor_streaming.json`
- [ ] T024 [US4] Verify accurate data flow from Isaac Sim to ROS 2 navigation stack using RViz and ROS 2 tooling
- [ ] T025 [US4] Configure Isaac Sim to receive and execute movement commands from the ROS 2 bipedal controller in `isaac_sim_configs/robot_control.json`
- [ ] T026 [US4] Validate correct robot movement within Isaac Sim based on Nav2 outputs

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T027 [P] Review and refine all Docusaurus documentation (`docs/robot-brain/**/*.md`)
- [ ] T028 Code cleanup and refactoring across ROS 2 nodes and configurations
- [ ] T029 Performance optimization of perception and navigation pipelines
- [ ] T030 Add comprehensive unit tests for custom ROS 2 nodes in `tests/unit/`
- [ ] T031 Implement robust error handling and logging across the system
- [ ] T032 Conduct integration tests for end-to-end autonomous navigation scenarios

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Integrates with US1 components (e.g., common ROS 2 setup) but should be independently testable.
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 and US2 components (e.g., shared VSLAM data) but should be independently testable.
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Depends on US1, US2, US3 for full perception and navigation stack, but simulation setup can begin in parallel.

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Stories 1, 2, 3 can theoretically begin in parallel (if team capacity allows, with careful management of shared resources and integration points). User Story 4's simulation setup can also run in parallel to the other stories, as long as the underlying components it tests are being developed.
- Different user stories can be worked on in parallel by different team members.

---

## Parallel Example: Initial Setup

```bash
# Setup Docusaurus and base project structure concurrently
Task: "Create base project structure for the robot control module in src/robot_brain/"
Task: "Initialize Docusaurus documentation project at docs/robot-brain/"
Task: "Create Docusaurus documentation directories: docs/robot-brain/perception/, docs/robot-brain/navigation/, docs/robot_brain/simulation/, docs/robot_brain/tutorials/"
Task: "Configure Docusaurus sidebars to reflect the planned documentation structure in docs/robot-brain/sidebars.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4 (can start simulation setup early, integrating with A, B, C's work as it becomes available)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence