# 📚 AI-Native Textbook Frontend

> Modern, interactive textbook platform for Physical AI & Humanoid Robotics built with Docusaurus 3.x

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.9.2-green.svg)](https://docusaurus.io/)
[![React](https://img.shields.io/badge/React-19.0.0-blue.svg)](https://reactjs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.9.3-blue.svg)](https://www.typescriptlang.org/)
[![Node](https://img.shields.io/badge/Node-%3E%3D20.0-brightgreen.svg)](https://nodejs.org/)

---

## 🎯 Overview

Interactive textbook platform featuring a comprehensive 13-week curriculum in Physical AI & Humanoid Robotics with an integrated RAG chatbot for real-time Q&A.

### Key Features

- **📖 Comprehensive Curriculum** - 26 chapters across 4 modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA systems
- **🤖 AI-Powered Chatbot** - Floating RAG chatbot widget for instant answers
- **🔍 Offline Search** - Client-side search with @easyops-cn/docusaurus-search-local plugin
- **🌐 Urdu Translation** - Real-time page translation to Urdu using MyMemory API
- **🤖 AI Integration** - One-click "Open in Claude" and "Open in ChatGPT" with page content
- **📋 Copy Page** - Copy entire page content or view as Markdown
- **🎨 Modern UI/UX** - Completely redesigned homepage with dark mode by default
- **♿ Accessibility** - WCAG-compliant with keyboard navigation
- **📱 Mobile-First** - Fully responsive across all devices
- **🚀 Fast Performance** - Static site generation with optimized assets
- **💻 Enhanced Code Highlighting** - VS Dark theme with 6+ programming languages

---

## 📁 Project Structure

```
my-website/
├── docs/                          # Textbook content (26 chapters)
│   ├── intro.md
│   ├── quickstart.md
│   ├── module-1-robotic-nervous-system/
│   │   ├── index.md
│   │   ├── week-1-introduction-to-physical-ai/
│   │   │   ├── chapter-1-what-is-physical-ai.md
│   │   │   └── chapter-2-lidar-imu-sensors.md
│   │   ├── week-2-ros-2-fundamentals/
│   │   │   ├── chapter-3-nodes-topics.md
│   │   │   └── chapter-4-services-packages.md
│   │   └── week-3-python-agent-integration/
│   │       ├── chapter-5-python-agent-integration.md
│   │       └── chapter-6-urdf-xacro-modeling.md
│   ├── module-2-digital-twin/
│   │   ├── index.md
│   │   ├── week-4-physics-simulation-in-gazebo/
│   │   │   ├── chapter-7-physics-simulation-fundamentals.md
│   │   │   └── chapter-8-gazebo-ros2-integration.md
│   │   └── week-5-high-fidelity-rendering-in-unity/
│   │       ├── chapter-9-unity-rendering-pipelines.md
│   │       └── chapter-10-sensor-simulation-unity.md
│   ├── module-3-ai-robot-brain/
│   │   ├── index.md
│   │   ├── week-6-nvidia-isaac-sim/
│   │   │   ├── chapter-11-isaac-sim-architecture.md
│   │   │   └── chapter-12-photorealistic-environments.md
│   │   ├── week-7-isaac-ros-hardware-accelerated/
│   │   │   ├── chapter-13-isaac-ros-vslam.md
│   │   │   └── chapter-14-nav2-integration.md
│   │   └── week-8-isaac-sim-reinforcement-learning/
│   │       ├── chapter-15-isaac-gym-gpu-rl.md
│   │       └── chapter-16-domain-randomization.md
│   └── module-4-vision-language-action/
│       ├── index.md
│       ├── week-9-voice-to-action-with-openai-whisper/
│       │   ├── chapter-17-whisper-speech-recognition.md
│       │   └── chapter-18-voice-ros2-integration.md
│       ├── week-10-cognitive-planning/
│       │   ├── chapter-19-llm-cognitive-planning.md
│       │   └── chapter-20-action-planning-safety.md
│       ├── week-11-system-integration/
│       │   ├── chapter-21-system-architecture.md
│       │   └── chapter-22-vision-multimodal.md
│       ├── week-12-advanced-deployment/
│       │   ├── chapter-23-real-world-deployment.md
│       │   └── chapter-24-learning-adaptation.md
│       └── week-13-testing-validation/
│           ├── chapter-25-testing-validation.md
│           └── chapter-26-final-deployment.md
├── src/
│   ├── components/
│   │   └── RagChatbot/            # Floating chatbot widget
│   ├── services/api/              # API integration layer
│   ├── pages/                     # Custom pages
│   ├── theme/                     # Theme customization
│   │   ├── Layout.jsx             # Custom layout wrapper
│   │   └── DocItem/               # Custom doc item theme
│   ├── css/                       # Global styles
│   └── utils/                     # Utility functions
├── static/                        # Static assets (images, favicon)
├── blog/                          # Blog posts
├── docusaurus.config.ts           # Main configuration
├── sidebars.ts                    # Sidebar navigation structure
├── tsconfig.json                  # TypeScript configuration
└── package.json                   # Dependencies and scripts
```

---

## 🛠️ Tech Stack

### Core Technologies

| Technology | Version | Purpose |
|------------|---------|---------|
| Docusaurus | 3.9.2 | Static site generator |
| React | 19.0.0 | UI library |
| TypeScript | 5.9.3 | Type-safe JavaScript |
| Node.js | ≥20.0 | Runtime environment |
| Infima CSS | Latest | Styling framework |
| MDX | 3.0.0 | Markdown with JSX |
| @easyops-cn/docusaurus-search-local | 0.55.1 | Offline search plugin |
| Lodash | 4.17.21 | Utility library |

---

## 🚀 Getting Started

### Prerequisites

- Node.js version 20.0 or higher
- npm or yarn package manager

### Installation

Navigate to the my-website directory and install dependencies using npm or yarn.

### Development

Start the development server to preview changes in real-time. The site will be available at localhost:3000 with hot reload enabled.

### Build

Create a production-ready build that generates static HTML, optimized JavaScript bundles, and minified CSS in the build directory.

### Deployment

Deploy to GitHub Pages using the built-in deployment command. Ensure you have proper repository access and configuration.

---

## 🎨 Key Components

### 1. Redesigned Homepage
**Location:** `src/pages/index.tsx`

Completely redesigned landing page with modern, professional design.

**Sections:**
- **Hero Section** - Eye-catching hero with curriculum stats (4 modules, 13 weeks, 26 chapters, 100+ code examples)
- **Tech Strip** - Visual showcase of 10 technologies covered (ROS 2, Gazebo, NVIDIA Isaac, Unity, etc.)
- **Module Cards** - Interactive cards for each of the 4 modules with detailed descriptions and topics
- **Features Section** - 6 feature cards highlighting textbook benefits
- **CTA Section** - Call-to-action encouraging users to start learning

### 2. Custom DocItem Layout
**Location:** `src/theme/DocItem/Layout/index.tsx`

Enhanced documentation page layout with powerful productivity features.

**Features:**

**Urdu Translation Button**
- Real-time translation of page content to Urdu
- Uses MyMemory Translation API
- Preserves code blocks and formatting
- Toggle between English and Urdu
- Loading state with Urdu text

**Copy Page Button**
- One-click copy of entire page content to clipboard
- Visual feedback on successful copy
- Useful for offline reading or note-taking

**AI Integration Dropdown**
- **View as Markdown** - Opens page content in new tab as plain text
- **Open in Claude** - Launches Claude.ai with page content pre-filled for Q&A
- **Open in ChatGPT** - Launches ChatGPT with page content pre-filled for Q&A
- Automatically truncates content to 3000 characters for API limits

### 3. Offline Search Plugin
**Location:** `docusaurus.config.ts`

Client-side search functionality using @easyops-cn/docusaurus-search-local.

**Configuration:**
- Hashed index for better performance
- Indexes all documentation pages
- Search bar positioned on right side
- Keyboard shortcut hints enabled
- Shows 8 results with 50-character context
- Highlights search terms on target pages
- Explicit search result paths

### 4. ChatWidget Component
**Location:** `src/components/RagChatbot/ChatWidget.tsx`

Floating AI chatbot providing instant answers about textbook content.

**Features:**
- Real-time message streaming with typing indicators
- Error handling with retry functionality
- Keyboard shortcuts for quick interaction
- Auto-scroll to latest messages
- Mobile-responsive design
- Accessibility support

### 5. Theme Customization

**Dark Mode by Default**
- Default color mode set to dark
- Respects user's system preferences
- Toggle available for light mode

**Enhanced Code Highlighting**
- VS Dark theme for code blocks
- Support for 6+ languages: bash, python, yaml, cpp, csharp, typescript
- Consistent syntax highlighting across light and dark modes

---

## 🔌 API Integration

### RAG Service
**Location:** `src/services/api/ragService.ts`

Handles communication with the RAG backend API for question answering functionality.

**Configuration:**
API configuration is managed in `src/services/api/config.ts` with settings for base URL, timeout, and retry logic.

**Endpoints:**
- POST /ask - Submit questions to RAG agent
- GET /health - Check API health status

---

## 📝 Content Management

### Adding New Content

1. Create Markdown files in the appropriate module directory under docs/
2. Add frontmatter with sidebar_position and title
3. Update sidebars.ts to include new content in navigation
4. Rebuild and redeploy the site

### Markdown Features

- Syntax-highlighted code blocks
- Admonitions for notes, tips, and warnings
- Tabs for multi-language examples
- MDX support for React components in Markdown

---

## 🎨 Styling

### Theme Customization

Customize colors, fonts, and spacing using CSS variables in `src/css/custom.css`. Supports both light and dark themes.

### Component Styles

Use CSS modules for component-scoped styling to avoid conflicts and maintain modularity.

---

## ♿ Accessibility

- Full keyboard navigation support
- ARIA labels and semantic HTML
- Focus management and visible indicators
- Screen reader compatibility
- WCAG AA color contrast compliance
- Reduced motion support for animations
- High contrast mode support

---

## 📱 Responsive Design

### Breakpoints

- Mobile: max-width 480px
- Tablet: max-width 768px
- Desktop: min-width 769px

### Mobile Optimizations

- Touch-friendly button sizes (minimum 44x44px)
- Simplified navigation for small screens
- Optimized chat widget layout
- Reduced animations for performance

---

## 🧪 Testing

### Type Checking

Run TypeScript type checking to catch type errors before deployment.

### Build Testing

Test the production build locally before deploying to ensure everything works correctly.

### Manual Testing

- Test all navigation links
- Verify chatbot functionality
- Check responsive design on different devices
- Test accessibility with keyboard navigation

---

## 📦 Build Output

Production build generates:
- Static HTML pages for all routes
- Optimized and code-split JavaScript bundles
- Minified CSS files
- Compressed and optimized images
- Service worker for offline support (optional)

Output directory: `build/`

---

## 🔧 Configuration Files

### docusaurus.config.ts

Main configuration file containing:
- Site metadata (title, tagline, URL)
- Deployment settings for GitHub Pages
- Theme configuration and customization
- Plugin settings and options
- Custom fields for environment variables

### sidebars.ts

Defines the sidebar navigation structure with categories and items for all curriculum modules.

### tsconfig.json

TypeScript configuration extending Docusaurus preset with custom compiler options.

---

## 🚨 Troubleshooting

### Build Failures

Clear cache and reinstall dependencies if build fails with module errors.

### Chatbot Connection Issues

- Verify API URL configuration
- Check CORS settings
- Confirm backend service is running
- Review browser console for errors

### Style Issues

Clear Docusaurus cache and restart development server if styles are not applying correctly.

---

## 📚 Documentation Resources

- [Docusaurus Official Documentation](https://docusaurus.io/docs)
- [React Documentation](https://react.dev/)
- [TypeScript Handbook](https://www.typescriptlang.org/docs/)
- [MDX Documentation](https://mdxjs.com/)

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

## 📄 License

Part of Hackathon I project for Panaversity.

---

**Built with ❤️ using Docusaurus**
