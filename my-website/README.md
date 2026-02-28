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

- **📖 Comprehensive Curriculum** - 4 modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA systems
- **🤖 AI-Powered Chatbot** - Floating RAG chatbot widget for instant answers
- **🎨 Modern UI/UX** - Responsive design with dark mode support
- **♿ Accessibility** - WCAG-compliant with keyboard navigation
- **📱 Mobile-First** - Fully responsive across all devices
- **🚀 Fast Performance** - Static site generation with optimized assets

---

## 📁 Project Structure

```
my-website/
├── docs/                          # Textbook content (Markdown)
│   ├── intro.md
│   ├── quickstart.md
│   ├── module-1-robotic-nervous-system/
│   ├── module-2-digital-twin/
│   ├── module-3-ai-robot-brain/
│   └── module-4-vision-language-action/
├── src/
│   ├── components/
│   │   ├── HomepageFeatures/      # Landing page components
│   │   └── RagChatbot/            # Floating chatbot widget
│   ├── services/api/              # API integration layer
│   ├── pages/                     # Custom pages
│   ├── theme/                     # Theme customization
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

### ChatWidget Component
**Location:** `src/components/RagChatbot/ChatWidget.tsx`

Floating AI chatbot providing instant answers about textbook content.

**Features:**
- Real-time message streaming with typing indicators
- Error handling with retry functionality
- Keyboard shortcuts for quick interaction
- Auto-scroll to latest messages
- Mobile-responsive design
- Accessibility support

### Homepage Component
**Location:** `src/pages/index.tsx`

Landing page featuring hero section, feature cards, and module overview with navigation to curriculum content.

### Custom Layout
**Location:** `src/theme/Layout.jsx`

Custom layout wrapper that integrates the floating chatbot widget across all pages.

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
