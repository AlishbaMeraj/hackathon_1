/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
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
          id: 'module-2/gazebo-physics-simulation',
          label: 'Gazebo Physics Simulation'
        },
        {
          type: 'doc',
          id: 'module-2/unity-visual-environments',
          label: 'Unity Visual Environments'
        },
        {
          type: 'doc',
          id: 'module-2/sensor-simulation-workflows',
          label: 'Sensor Simulation Workflows'
        }
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        {
          type: 'doc',
          id: 'ai-robot-brain/intro',
          label: 'Introduction to AI-Robot Brain'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/nvidia-isaac-sim',
          label: 'NVIDIA Isaac Sim'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/synthetic-data-generation',
          label: 'Synthetic Data Generation'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/training-ready-environments',
          label: 'Training-Ready Environments'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/hardware-accelerated-perception',
          label: 'Hardware-Accelerated Perception'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/vslam-concepts',
          label: 'Visual SLAM Concepts'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/sensor-pipeline-acceleration',
          label: 'Sensor Pipeline Acceleration'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/path-planning-concepts',
          label: 'Path Planning Concepts'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/humanoid-navigation-challenges',
          label: 'Humanoid Navigation Challenges'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/sim-to-real-transfer',
          label: 'Sim-to-Real Transfer'
        },
        {
          type: 'doc',
          id: 'ai-robot-brain/conclusion',
          label: 'Conclusion'
        }
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;