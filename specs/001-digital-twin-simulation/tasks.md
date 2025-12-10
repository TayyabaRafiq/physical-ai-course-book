# Module 2 Tasks: The Digital Twin (Gazebo & Unity)

**Feature Branch**: `001-digital-twin-simulation`
**Created**: 2025-12-05
**Input**: Approved plan from `/specs/001-digital-twin-simulation/plan.md`

## Phase 1: Setup and Boilerplate Creation

- [ ] T001 Create Docusaurus documentation directory for Module 2: `docs/02-digital-twin-simulation/`
- [ ] T002 [P] Create Docusaurus Markdown file for Simulation Engines: `docs/02-digital-twin-simulation/01-simulation-engines.md`
- [ ] T003 [P] Create Docusaurus Markdown file for Gazebo Physics: `docs/02-digital-twin-simulation/02-gazebo-physics.md`
- [ ] T004 [P] Create Docusaurus Markdown file for Unity HRI: `docs/02-digital-twin-simulation/03-unity-hri.md`
- [ ] T005 [P] Create Docusaurus Markdown file for Simulated Sensors: `docs/02-digital-twin-simulation/04-simulated-sensors.md`
- [ ] T006 [P] Create Docusaurus category configuration for Module 2: `docs/02-digital-twin-simulation/_category_.json`
- [ ] T007 Create ROS 2 Python boilerplate package directory for `ros2_sim_data_publisher`: `src/02-digital-twin-simulation/ros2_sim_data_publisher/`
- [ ] T008 [P] Create `setup.py` for `ros2_sim_data_publisher`: `src/02-digital-twin-simulation/ros2_sim_data_publisher/setup.py`
- [ ] T009 [P] Create `package.xml` for `ros2_sim_data_publisher`: `src/02-digital-twin-simulation/ros2_sim_data_publisher/package.xml`
- [ ] T010 [P] Create `__init__.py` for `ros2_sim_data_publisher` Python package: `src/02-digital-twin-simulation/ros2_sim_data_publisher/ros2_sim_data_publisher/__init__.py`
- [ ] T011 [P] Create `sensor_publisher_node.py` for `ros2_sim_data_publisher`: `src/02-digital-twin-simulation/ros2_sim_data_publisher/ros2_sim_data_publisher/sensor_publisher_node.py`
- [ ] T012 [P] Create `launch/sim_data_launch.py` for `ros2_sim_data_publisher`: `src/02-digital-twin-simulation/ros2_sim_data_publisher/launch/sim_data_launch.py`
- [ ] T013 Create Gazebo assets directory: `src/02-digital-twin-simulation/gazebo_assets/`
- [ ] T014 [P] Create Gazebo models directory: `src/02-digital-twin-simulation/gazebo_assets/models/`
- [ ] T015 [P] Create example robot model directory: `src/02-digital-twin-simulation/gazebo_assets/models/simple_robot/`
- [ ] T016 [P] Create `model.sdf` for simple robot in Gazebo: `src/02-digital-twin-simulation/gazebo_assets/models/simple_robot/model.sdf`
- [ ] T017 [P] Create `model.config` for simple robot in Gazebo: `src/02-digital-twin-simulation/gazebo_assets/models/simple_robot/model.config`
- [ ] T018 [P] Create Gazebo worlds directory: `src/02-digital-twin-simulation/gazebo_assets/worlds/`
- [ ] T019 [P] Create `empty.world` for Gazebo: `src/02-digital-twin-simulation/gazebo_assets/worlds/empty.world`
- [ ] T020 Create Unity project directory: `src/02-digital-twin-simulation/unity_project/`
- [ ] T021 [P] Create Unity Assets directory: `src/02-digital-twin-simulation/unity_project/Assets/`
- [ ] T022 [P] Create Unity Scenes directory: `src/02-digital-twin-simulation/unity_project/Assets/Scenes/`
- [ ] T023 [P] Create `MainScene.unity` in Unity: `src/02-digital-twin-simulation/unity_project/Assets/Scenes/MainScene.unity`
- [ ] T024 [P] Create Unity Scripts directory: `src/02-digital-twin-simulation/unity_project/Assets/Scripts/`
- [ ] T025 [P] Create `RosConnection.cs` in Unity: `src/02-digital-twin-simulation/unity_project/Assets/Scripts/RosConnection.cs`
- [ ] T026 [P] Create Unity Materials directory: `src/02-digital-twin-simulation/unity_project/Assets/Materials/`
- [ ] T027 [P] Create `RobotMaterial.mat` in Unity: `src/02-digital-twin-simulation/unity_project/Assets/Materials/RobotMaterial.mat`
- [ ] T028 Create or update Docusaurus `sidebars.js` configuration in project root.

## Phase 2: User Story 1 - Understand Simulation Engine Trade-offs (Priority: P1)

**Story Goal**: Learners will gain a clear understanding of the trade-offs between Gazebo (physics-focused) and Unity (high-fidelity rendering/interactivity) for robotics simulation. They will be able to articulate appropriate use-cases for each engine.
**Independent Test**: Can be fully tested by learners correctly identifying the strengths and weaknesses of Gazebo and Unity for different simulation scenarios.

- [ ] T029 [US1] Enhance `docs/02-digital-twin-simulation/01-simulation-engines.md` with detailed explanations of Gazebo and Unity trade-offs, use cases, and supporting diagrams.

## Phase 3: User Story 2 - Implement Gazebo Physics Simulation (Priority: P1)

**Story Goal**: Learners will be able to implement fundamental physics-based simulation concepts (gravity, collisions) within a Gazebo environment. They will define a `.world` file with various shapes and verify accurate physics interactions.
**Independent Test**: Can be fully tested by successfully deploying a Gazebo `.world` file and verifying accurate physics behavior of objects and a robot model within it.

- [ ] T030 [US2] Enhance `docs/02-digital-twin-simulation/02-gazebo-physics.md` with explanations of Gazebo physics concepts (gravity, collisions) and embed the content of `src/02-digital-twin-simulation/gazebo_assets/models/simple_robot/model.sdf` and `src/02-digital-twin-simulation/gazebo_assets/worlds/empty.world`.
- [ ] T031 [US2] Add step-by-step instructions for creating and running the Gazebo environment in `docs/02-digital-twin-simulation/02-gazebo-physics.md`, including necessary CLI commands.

## Phase 4: User Story 3 - Implement Unity High-Fidelity Rendering & HRI (Priority: P1)

**Story Goal**: Learners will be able to implement high-fidelity rendering and interactivity concepts in Unity. They will duplicate the Gazebo environment layout with realistic materials, lighting, and a simple Human-Robot Interaction (HRI) scenario.
**Independent Test**: Can be fully tested by successfully deploying a Unity Scene with realistic rendering and demonstrating a functional HRI scenario.

- [ ] T032 [US3] Enhance `docs/02-digital-twin-simulation/03-unity-hri.md` with explanations of Unity rendering, PBR materials, lighting, post-processing, and HRI concepts.
- [ ] T033 [US3] Embed conceptual Unity code snippets (e.g., `RosConnection.cs` and C# scripts for HRI) into `docs/02-digital-twin-simulation/03-unity-hri.md`.
- [ ] T034 [US3] Add step-by-step instructions for setting up the Unity scene and implementing a simple HRI scenario in `docs/02-digital-twin-simulation/03-unity-hri.md`.

## Phase 5: User Story 4 - Define & Integrate Simulated Sensors (Priority: P2)

**Story Goal**: Learners will be able to define and integrate three common robot sensors (LiDAR, Depth Camera, IMU) into the digital twin model and understand their simulated output. They will be able to visualize and plot sensor data in real-time.
**Independent Test**: Can be fully tested by successfully integrating and visualizing data from simulated LiDAR, Depth Camera, and IMU sensors.

- [ ] T035 [US4] Enhance `docs/02-digital-twin-simulation/04-simulated-sensors.md` with explanations of LiDAR, Depth Camera, and IMU sensor principles and their simulation in Gazebo and Unity.
- [ ] T036 [US4] Embed conceptual code snippets for `src/02-digital-twin-simulation/ros2_sim_data_publisher/ros2_sim_data_publisher/sensor_publisher_node.py` and the ROS 2 launch file `src/02-digital-twin-simulation/ros2_sim_data_publisher/launch/sim_data_launch.py`.
- [ ] T037 [US4] Add step-by-step instructions for configuring and running the simulated sensors, including ROS 2 CLI commands and visualization instructions (e.g., RVIZ, Unity debug views) in `docs/02-digital-twin-simulation/04-simulated-sensors.md`.

## Dependencies

- Phase 1 (Setup and Boilerplate Creation) must be completed before any User Story phases.
- User Stories 1, 2, and 3 can be worked on largely in parallel after Phase 1 is complete.
- User Story 4 depends on aspects of User Stories 2 and 3 for sensor integration into Gazebo and Unity, respectively.

## Parallel Execution Opportunities

- Tasks T002 to T006 (Docusaurus file creation) can be done in parallel.
- Tasks T008 to T012 (ROS 2 boilerplate file creation) can be done in parallel.
- Tasks T014 to T019 (Gazebo asset/world creation) can be done in parallel.
- Tasks T021 to T027 (Unity project structure/asset creation) can be done in parallel.
- User Story 1, User Story 2, and User Story 3 can be worked on largely in parallel after Phase 1 is complete.

## Implementation Strategy

- Focus on an MVP first, ensuring all Docusaurus content files and basic ROS 2 Python, Gazebo, and Unity boilerplate are correctly structured.
- Incrementally deliver each user story, ensuring each one is independently testable before moving to the next.
- Prioritize P1 user stories (Understand Simulation Engine Trade-offs, Implement Gazebo Physics Simulation, Implement Unity High-Fidelity Rendering & HRI) before P2 (Define & Integrate Simulated Sensors).
