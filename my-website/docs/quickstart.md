# Quick Start Guide

This guide provides a fast path to getting the AI-Native Textbook up and running for both reading and development.

## For Readers

### Reading Online
The textbook is deployed at: https://muhammadwaheedairi.github.io/hackathon_textbook_ai_robotics/

Simply visit the link to start reading the complete 13-week curriculum covering:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)

### Reading Locally
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

4. Start the development server:
   ```bash
   npm start
   ```

5. Open your browser to `http://localhost:3000` to view the textbook

## For Content Developers

### Prerequisites
- Node.js 18+ LTS (recommended)
- npm package manager
- Git for version control

### Setup Process
1. Install Node.js from [nodejs.org](https://nodejs.org/)
2. Clone the repository
3. Install dependencies with `npm install`
4. Start development with `npm start`

### Adding New Content
1. Create a new Markdown file in the appropriate module directory under `my-website/docs/`
2. Follow the existing file naming convention (e.g., `week-1-introduction-to-physical-ai.md`)
3. Include proper Docusaurus frontmatter:
   ```markdown
   ---
   title: Your Week Title
   sidebar_label: Week Title
   sidebar_position: X
   ---
   ```
4. Update `my-website/sidebars.ts` to include your new content in the navigation
5. Test locally before committing

### Content Structure
The textbook follows a strict 4-module, 13-week curriculum:
- **Module 1** (Weeks 1-3): ROS 2 fundamentals and physical AI
- **Module 2** (Weeks 4-5): Gazebo simulation and Unity
- **Module 3** (Weeks 6-8): NVIDIA Isaac and advanced AI
- **Module 4** (Weeks 9-13): Vision-Language-Action and capstone

## Deployment

The textbook is automatically deployed to GitHub Pages when changes are pushed to the main branch. The deployment workflow is defined in `.github/workflows/deploy.yml` and configured in `docusaurus.config.ts`.

## Troubleshooting

### Common Issues

**Build fails with memory errors:**
- Increase Node.js memory limit: `export NODE_OPTIONS="--max_old_space_size=4096"`

**Local server doesn't start:**
- Clear Docusaurus cache: `npx docusaurus clear`
- Reinstall dependencies: `rm -rf node_modules && npm install`

**Navigation not showing new content:**
- Verify the new file is referenced in `sidebars.ts`
- Check that the file has proper frontmatter
- Restart the development server

### Getting Help
- Check the full README.md for detailed instructions
- Open an issue in the GitHub repository for problems
- Review the Docusaurus documentation for platform-specific issues

## Next Steps

1. **Explore the curriculum** - Start with Module 1, Week 1
2. **Try hands-on exercises** - Each week includes practical exercises
3. **Contribute improvements** - Submit pull requests for content enhancements
4. **Join the community** - Engage with other learners and educators