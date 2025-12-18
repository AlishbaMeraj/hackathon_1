// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'doc',
          id: 'module-1/intro-to-ros2',
          label: 'Introduction to ROS 2 for Physical AI'
        },
        {
          type: 'doc',
          id: 'module-1/ros2-communication',
          label: 'ROS 2 Communication Model'
        },
        {
          type: 'doc',
          id: 'module-1/urdf-robot-structure',
          label: 'Robot Structure with URDF'
        }
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Physics Simulation and Visual Environments',
      items: [
        {
          type: 'doc',
          id: 'module-2/foundational-concepts',
          label: 'Foundational Concepts'
        },
        {
          type: 'doc',
          id: 'module-2/gazebo-physics-simulation',
          label: 'Gazebo Physics Simulation'
        },
        {
          type: 'doc',
          id: 'module-2/gazebo-practical-examples',
          label: 'Gazebo Practical Examples'
        },
        {
          type: 'doc',
          id: 'module-2/unity-visual-simulation',
          label: 'Unity Visual Simulation'
        },
        {
          type: 'doc',
          id: 'module-2/unity-human-robot-interaction',
          label: 'Human-Robot Interaction in Unity'
        },
        {
          type: 'doc',
          id: 'module-2/sensor-simulation-overview',
          label: 'Sensor Simulation Overview'
        },
        {
          type: 'doc',
          id: 'module-2/sim-to-real-transfer',
          label: 'Sim-to-Real Transfer Challenges'
        },
        {
          type: 'doc',
          id: 'module-2/sensor-fusion-examples',
          label: 'Sensor Fusion Examples'
        },
        {
          type: 'doc',
          id: 'module-2/module-conclusion',
          label: 'Module Conclusion'
        }
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI and Machine Learning for Humanoid Robots',
      items: [
        {
          type: 'doc',
          id: 'module-3/ai-integration-fundamentals',
          label: 'AI Integration Fundamentals'
        },
        {
          type: 'doc',
          id: 'module-3/perception-systems',
          label: 'Perception Systems for Humanoid Robots'
        },
        {
          type: 'doc',
          id: 'module-3/behavior-trees',
          label: 'Behavior Trees and Decision Making'
        },
        {
          type: 'doc',
          id: 'module-3/reinforcement-learning',
          label: 'Reinforcement Learning Applications'
        },
        {
          type: 'doc',
          id: 'module-3/nlp-humanoid-interaction',
          label: 'Natural Language Processing'
        },
        {
          type: 'doc',
          id: 'module-3/computer-vision',
          label: 'Computer Vision for Humanoid Robots'
        },
        {
          type: 'doc',
          id: 'module-3/ai-motion-planning',
          label: 'Motion Planning with AI'
        },
        {
          type: 'doc',
          id: 'module-3/ai-safety-ethics',
          label: 'AI Safety and Ethics'
        }
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) for Humanoid Robots',
      items: [
        {
          type: 'doc',
          id: 'module-4/voice-to-action',
          label: 'Voice-to-Action Systems'
        },
        {
          type: 'doc',
          id: 'module-4/cognitive-planning-with-llms',
          label: 'Cognitive Planning with LLMs'
        },
        {
          type: 'doc',
          id: 'module-4/autonomous-humanoid-capstone',
          label: 'Autonomous Humanoid Capstone'
        }
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
