# AI-Native Textbook — Physical AI & Humanoid Robotics

A comprehensive textbook covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems for physical AI and humanoid robotics applications. This textbook is designed as a 13-week curriculum organized into 4 modules, providing students and educators with a structured learning path through cutting-edge robotics technologies.

## Curriculum Overview

### Module 1 — The Robotic Nervous System (ROS 2)
- **Week 1**: Introduction to Physical AI and Sensors (LIDAR, IMUs)
- **Week 2**: ROS 2 Fundamentals — Nodes, Topics, Services, Packages
- **Week 3**: Python Agent Integration with ROS Controllers + URDF Modeling

### Module 2 — The Digital Twin (Gazebo & Unity)
- **Week 4**: Physics Simulation in Gazebo — Gravity, Collisions
- **Week 5**: High-Fidelity Rendering in Unity + Sensor Simulation

### Module 3 — The AI-Robot Brain (NVIDIA Isaac™)
- **Week 6**: NVIDIA Isaac Sim — Photorealistic Simulation
- **Week 7**: Isaac ROS — Hardware-Accelerated VSLAM + Nav2
- **Week 8**: Isaac Sim for Reinforcement Learning — Advanced Tooling

### Module 4 — Vision-Language-Action (VLA)
- **Week 9**: Voice-to-Action with OpenAI Whisper
- **Week 10**: Cognitive Planning — LLMs Translating Natural Language to ROS 2 Actions
- **Weeks 11-13**: Capstone — Autonomous Humanoid Deployment & Testing

## Setup Instructions

### Prerequisites

- Node.js 18+ LTS (recommended)
- npm package manager
- Git for version control

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics.git
   ```

2. Navigate to the website directory:
   ```bash
   cd hackathon_textbook_ai_robotics/my-website
   ```

3. Install dependencies:
   ```bash
   npm install
   ```

### Local Development

1. Start the development server:
   ```bash
   npm start
   ```

2. Open your browser to [http://localhost:3000](http://localhost:3000) to view the textbook

### Build for Production

To build the static site for production:

```bash
npm run build
```

The built site will be available in the `build/` directory.

### Deployment

This project is configured for GitHub Pages deployment. The site will automatically deploy when changes are pushed to the main branch using the GitHub Actions workflow in `.github/workflows/deploy.yml`.

## Contributing

To contribute content to the textbook:

1. Create or edit Markdown files in the `my-website/docs/` directory following the module/week structure
2. Update the sidebar configuration in `my-website/sidebars.ts` if adding new content
3. Test your changes locally before submitting a pull request

## Technologies Used

- [Docusaurus](https://docusaurus.io/) - Static site generator for documentation
- [React](https://reactjs.org/) - JavaScript library for building user interfaces
- [GitHub Pages](https://pages.github.com/) - Static site hosting
- [GitHub Actions](https://github.com/features/actions) - Continuous deployment

## License

This textbook content is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

## Support

For questions or support, please open an issue in the GitHub repository.