import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    'quickstart',
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1-robotic-nervous-system/index',
        {
          type: 'category',
          label: 'Week 1: Introduction to Physical AI',
          collapsible: true,
          collapsed: true,
          items: [
            'module-1-robotic-nervous-system/week-1-introduction-to-physical-ai/chapter-1-what-is-physical-ai',
            'module-1-robotic-nervous-system/week-1-introduction-to-physical-ai/chapter-2-lidar-imu-sensors',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: ROS 2 Fundamentals',
          collapsible: true,
          collapsed: true,
          items: [
            'module-1-robotic-nervous-system/week-2-ros-2-fundamentals/chapter-3-nodes-topics',
            'module-1-robotic-nervous-system/week-2-ros-2-fundamentals/chapter-4-services-packages',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Python Agent Integration',
          collapsible: true,
          collapsed: true,
          items: [
            'module-1-robotic-nervous-system/week-3-python-agent-integration/chapter-5-python-agent-integration',
            'module-1-robotic-nervous-system/week-3-python-agent-integration/chapter-6-urdf-xacro-modeling',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2 — The Digital Twin',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2-digital-twin/index',
        {
          type: 'category',
          label: 'Week 4: Physics Simulation in Gazebo',
          collapsible: true,
          collapsed: true,
          items: [
            'module-2-digital-twin/week-4-physics-simulation-in-gazebo/chapter-7-physics-simulation-fundamentals',
            'module-2-digital-twin/week-4-physics-simulation-in-gazebo/chapter-8-gazebo-ros2-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 5: High-Fidelity Rendering in Unity',
          collapsible: true,
          collapsed: true,
          items: [
            'module-2-digital-twin/week-5-high-fidelity-rendering-in-unity/chapter-9-unity-rendering-pipelines',
            'module-2-digital-twin/week-5-high-fidelity-rendering-in-unity/chapter-10-sensor-simulation-unity',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3 — The AI-Robot Brain',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3-ai-robot-brain/index',
        {
          type: 'category',
          label: 'Week 6: NVIDIA Isaac Sim',
          collapsible: true,
          collapsed: true,
          items: [
            'module-3-ai-robot-brain/week-6-nvidia-isaac-sim/chapter-11-isaac-sim-architecture',
            'module-3-ai-robot-brain/week-6-nvidia-isaac-sim/chapter-12-photorealistic-environments',
          ],
        },
        {
          type: 'category',
          label: 'Week 7: Isaac ROS Hardware Accelerated',
          collapsible: true,
          collapsed: true,
          items: [
            'module-3-ai-robot-brain/week-7-isaac-ros-hardware-accelerated/chapter-13-isaac-ros-vslam',
            'module-3-ai-robot-brain/week-7-isaac-ros-hardware-accelerated/chapter-14-nav2-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 8: Isaac Sim Reinforcement Learning',
          collapsible: true,
          collapsed: true,
          items: [
            'module-3-ai-robot-brain/week-8-isaac-sim-reinforcement-learning/chapter-15-isaac-gym-gpu-rl',
            'module-3-ai-robot-brain/week-8-isaac-sim-reinforcement-learning/chapter-16-domain-randomization',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4-vision-language-action/index',
        {
          type: 'category',
          label: 'Week 9: Voice-to-Action with Whisper',
          collapsible: true,
          collapsed: true,
          items: [
            'module-4-vision-language-action/week-9-voice-to-action-with-openai-whisper/chapter-17-whisper-speech-recognition',
            'module-4-vision-language-action/week-9-voice-to-action-with-openai-whisper/chapter-18-voice-ros2-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 10: Cognitive Planning with LLMs',
          collapsible: true,
          collapsed: true,
          items: [
            'module-4-vision-language-action/week-10-cognitive-planning/chapter-19-llm-cognitive-planning',
            'module-4-vision-language-action/week-10-cognitive-planning/chapter-20-action-planning-safety',
          ],
        },
        {
          type: 'category',
          label: 'Week 11: System Integration',
          collapsible: true,
          collapsed: true,
          items: [
            'module-4-vision-language-action/week-11-system-integration/chapter-21-system-architecture',
            'module-4-vision-language-action/week-11-system-integration/chapter-22-vision-multimodal',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Advanced Deployment',
          collapsible: true,
          collapsed: true,
          items: [
            'module-4-vision-language-action/week-12-advanced-deployment/chapter-23-real-world-deployment',
            'module-4-vision-language-action/week-12-advanced-deployment/chapter-24-learning-adaptation',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Testing & Validation',
          collapsible: true,
          collapsed: true,
          items: [
            'module-4-vision-language-action/week-13-testing-validation/chapter-25-testing-validation',
            'module-4-vision-language-action/week-13-testing-validation/chapter-26-final-deployment',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
