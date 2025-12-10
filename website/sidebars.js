// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 *
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'generated-index',
        title: 'The Robotic Nervous System (ROS 2)',
        description: 'Learn the fundamentals of ROS 2, the Robot Operating System. Master Python interfacing with rclpy and URDF for robot modeling. Note: ROS 2 is optional for the VLA MVP - the system works with Pure Python execution.',
        slug: '/module-1-ros2-fundamentals',
      },
      items: [
        'robot-nervous-system/ros2-fundamentals',
        'robot-nervous-system/rclpy-bridge',
        'robot-nervous-system/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      link: {
        type: 'generated-index',
        title: 'Gazebo & Unity Integration',
        description: 'Master digital twin concepts using Gazebo for physics simulation and Unity for high-fidelity visualization. Learn to integrate simulated sensors with ROS 2 for realistic robot testing.',
        slug: '/module-2-digital-twin',
      },
      items: [
        'digital-twin-simulation/simulation-engines',
        'digital-twin-simulation/gazebo-physics',
        'digital-twin-simulation/unity-hri',
        'digital-twin-simulation/simulated-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac Sim',
      link: {
        type: 'generated-index',
        title: 'AI-Robot Brain: GPU-Accelerated Perception',
        description: 'Leverage NVIDIA Isaac Sim and Isaac ROS for high-performance robot perception and autonomous navigation. Integrate VSLAM, Nav2, and GPU-accelerated processing for intelligent bipedal movement.',
        slug: '/module-3-isaac-sim',
      },
      items: [
        'isaac-robot-brain/isaac-sim-setup',
        'isaac-robot-brain/isaac-ros-vslam',
        'isaac-robot-brain/nav2-integration',
        'isaac-robot-brain/autonomous-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Integration',
      link: {
        type: 'generated-index',
        title: 'Vision-Language-Action Pipeline',
        description: 'Converge LLMs and Robotics through a multi-modal voice-to-action pipeline. Use OpenAI Whisper for speech recognition, LLM-based cognitive planning, and Pure Python execution for natural language robot control. ROS 2 integration available as optional extension.',
        slug: '/module-4-vla-integration',
      },
      items: [
        'vla-integration/voice-interface',
        'vla-integration/cognitive-planning',
        'vla-integration/python-execution',
        'vla-integration/end-to-end-pipeline',
        'vla-integration/ros-extension',
      ],
    },
  ],
};

export default sidebars;