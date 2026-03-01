# 🤖 AI-Native Textbook: Physical AI & Humanoid Robotics

> A comprehensive, interactive textbook platform with integrated RAG chatbot for teaching Physical AI & Humanoid Robotics

[![Live Demo](https://img.shields.io/badge/Live-Demo-success.svg)](https://muhammadwaheedairi.github.io/hackathon_textbook_ai_robotics/)
[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.9.2-green.svg)](https://docusaurus.io/)
[![FastAPI](https://img.shields.io/badge/FastAPI-Latest-009688.svg)](https://fastapi.tiangolo.com/)
[![Python](https://img.shields.io/badge/Python-3.11+-blue.svg)](https://www.python.org/)
[![React](https://img.shields.io/badge/React-19.0.0-61dafb.svg)](https://reactjs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.9.3-blue.svg)](https://www.typescriptlang.org/)

---

## 🧭 Quick Navigation

| Section | Description | Link |
|---------|-------------|------|
| 🎨 **Frontend** | Docusaurus textbook platform with React components | [my-website/](./my-website/) · [README](./my-website/README.md) |
| ⚙️ **Backend** | FastAPI RAG chatbot with vector search | [backend/](./backend/) · [README](./backend/README.md) |
| 📋 **Specs** | Feature specifications and implementation plans | [specs/](./specs/) |
| 📚 **History** | Architecture Decision Records (ADRs) and Prompt History | [history/](./history/) |
| 🔧 **.specify** | Spec-Kit Plus configuration and project memory | [.specify/](./.specify/) |

---

## 📖 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Architecture](#-architecture)
- [Project Structure](#-project-structure)
- [Technology Stack](#-technology-stack)
- [Getting Started](#-getting-started)
- [Development Workflow](#-development-workflow)
- [Deployment](#-deployment)
- [API Documentation](#-api-documentation)
- [Content Management](#-content-management)
- [Spec-Driven Development](#-spec-driven-development)
- [Testing](#-testing)
- [Performance](#-performance)
- [Security](#-security)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)

---

## 🎯 Overview

This project is an AI-native textbook platform built for **Hackathon I** at Panaversity. It delivers a comprehensive 13-week curriculum covering Physical AI & Humanoid Robotics, featuring an intelligent RAG (Retrieval-Augmented Generation) chatbot that answers questions about the textbook content in real-time.

### What Makes This Special?

- **AI-First Learning Experience** - Students can ask questions and get instant, contextual answers
- **Modern Web Technologies** - Built with cutting-edge frameworks for optimal performance
- **Production-Ready** - Deployed on GitHub Pages (frontend) and Hugging Face Spaces (backend)
- **Comprehensive Curriculum** - 13 weeks of structured content across 4 major modules
- **Accessible & Responsive** - Works seamlessly on all devices with full accessibility support
- **Spec-Driven Development** - Built using Spec-Kit Plus methodology for maintainability

### Course Curriculum

**Module 1: The Robotic Nervous System (ROS 2)** - Weeks 1-3 (6 Chapters)
- Week 1: Introduction to Physical AI
  - Chapter 1: What is Physical AI
  - Chapter 2: LiDAR & IMU Sensors
- Week 2: ROS 2 Fundamentals
  - Chapter 3: Nodes & Topics
  - Chapter 4: Services & Packages
- Week 3: Python Agent Integration
  - Chapter 5: Python Agent Integration
  - Chapter 6: URDF & Xacro Modeling

**Module 2: The Digital Twin (Gazebo & Unity)** - Weeks 4-5 (4 Chapters)
- Week 4: Physics Simulation in Gazebo
  - Chapter 7: Physics Simulation Fundamentals
  - Chapter 8: Gazebo ROS2 Integration
- Week 5: High-Fidelity Rendering in Unity
  - Chapter 9: Unity Rendering Pipelines
  - Chapter 10: Sensor Simulation in Unity

**Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - Weeks 6-8 (6 Chapters)
- Week 6: NVIDIA Isaac Sim
  - Chapter 11: Isaac Sim Architecture
  - Chapter 12: Photorealistic Environments
- Week 7: Isaac ROS Hardware Accelerated
  - Chapter 13: Isaac ROS VSLAM
  - Chapter 14: Nav2 Integration
- Week 8: Isaac Sim Reinforcement Learning
  - Chapter 15: Isaac Gym GPU RL
  - Chapter 16: Domain Randomization

**Module 4: Vision-Language-Action (VLA)** - Weeks 9-13 (10 Chapters)
- Week 9: Voice-to-Action with OpenAI Whisper
  - Chapter 17: Whisper Speech Recognition
  - Chapter 18: Voice ROS2 Integration
- Week 10: Cognitive Planning with LLMs
  - Chapter 19: LLM Cognitive Planning
  - Chapter 20: Action Planning & Safety
- Week 11: System Integration
  - Chapter 21: System Architecture
  - Chapter 22: Vision & Multimodal
- Week 12: Advanced Deployment
  - Chapter 23: Real-World Deployment
  - Chapter 24: Learning & Adaptation
- Week 13: Testing & Validation
  - Chapter 25: Testing & Validation
  - Chapter 26: Final Deployment

---

## ✨ Features

### Frontend Features

- **📚 Interactive Textbook** - 26 chapters with rich Markdown content and code highlighting
- **🤖 Floating AI Chatbot** - Always-accessible RAG chatbot widget for instant Q&A
- **🔍 Offline Search** - Client-side search with instant results using local indexing
- **🌐 Urdu Translation** - Real-time page translation to Urdu with one-click toggle
- **🤖 AI Integration** - Direct integration with Claude and ChatGPT for page-specific questions
- **📋 Smart Copy** - Copy entire pages or view as Markdown with dropdown options
- **🎨 Modern UI/UX** - Redesigned homepage with hero, stats, module cards, and features
- **🌙 Dark Mode Default** - Beautiful dark theme by default with light mode toggle
- **📱 Fully Responsive** - Optimized for mobile, tablet, and desktop with touch-friendly controls
- **♿ Accessibility** - WCAG AA compliant with keyboard navigation and screen reader support
- **🔍 Enhanced Search** - Search bar with keyboard shortcuts and highlighted results
- **💻 Code Highlighting** - VS Dark theme supporting 6+ programming languages

### Backend Features

- **🔍 Semantic Search** - Vector similarity search using Cohere embeddings
- **🧠 LLM-Powered Answers** - Context-aware responses via OpenRouter
- **⚡ Fast API** - Async FastAPI with sub-second response times
- **📊 Metadata Tracking** - Query time, confidence scores, source URLs
- **🔄 Auto-Embedding Pipeline** - Automated content ingestion from live site
- **🐳 Docker Support** - Containerized for easy deployment
- **🌐 CORS Enabled** - Cross-origin requests support

---

## 🏗️ Architecture

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         User Browser                         │
└────────────────────────┬────────────────────────────────────┘
                         │
                         │ HTTPS
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              Frontend (GitHub Pages)                         │
│  - Docusaurus 3.x + React 19                                │
│  - Static Site Generation                                   │
│  - ChatWidget Component                                     │
└────────────────────────┬────────────────────────────────────┘
                         │
                         │ REST API (CORS)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│           Backend (Hugging Face Spaces)                      │
│  - FastAPI + Uvicorn                                        │
│  - RAG Agent + Retriever                                    │
│  - Cohere Embeddings + OpenRouter LLM                       │
└─────────────────────────────────────┬───────────────────────┘
                                      │
                                      │ Vector Search
                                      ▼
                            ┌─────────────────────┐
                            │   Qdrant Cloud      │
                            │  Vector Database    │
                            └─────────────────────┘
```

### Data Flow

1. User Query → Frontend ChatWidget
2. API Request → Backend FastAPI endpoint
3. Embedding Generation → Cohere API converts query to vector
4. Vector Search → Qdrant finds similar content chunks
5. Context Building → Retrieved chunks formatted as context
6. LLM Generation → OpenRouter generates answer
7. Response → Formatted JSON with answer, sources, metadata
8. Display → ChatWidget shows answer to user

---

## 📁 Project Structure

```
hackathon_textbook_ai_robotics/
├── my-website/                    # Frontend (Docusaurus)
│   ├── docs/                      # Textbook content (26 chapters)
│   │   ├── intro.md
│   │   ├── quickstart.md
│   │   ├── module-1-robotic-nervous-system/
│   │   │   ├── index.md
│   │   │   ├── week-1-introduction-to-physical-ai/
│   │   │   │   ├── chapter-1-what-is-physical-ai.md
│   │   │   │   └── chapter-2-lidar-imu-sensors.md
│   │   │   ├── week-2-ros-2-fundamentals/
│   │   │   │   ├── chapter-3-nodes-topics.md
│   │   │   │   └── chapter-4-services-packages.md
│   │   │   └── week-3-python-agent-integration/
│   │   │       ├── chapter-5-python-agent-integration.md
│   │   │       └── chapter-6-urdf-xacro-modeling.md
│   │   ├── module-2-digital-twin/
│   │   │   ├── index.md
│   │   │   ├── week-4-physics-simulation-in-gazebo/
│   │   │   │   ├── chapter-7-physics-simulation-fundamentals.md
│   │   │   │   └── chapter-8-gazebo-ros2-integration.md
│   │   │   └── week-5-high-fidelity-rendering-in-unity/
│   │   │       ├── chapter-9-unity-rendering-pipelines.md
│   │   │       └── chapter-10-sensor-simulation-unity.md
│   │   ├── module-3-ai-robot-brain/
│   │   │   ├── index.md
│   │   │   ├── week-6-nvidia-isaac-sim/
│   │   │   │   ├── chapter-11-isaac-sim-architecture.md
│   │   │   │   └── chapter-12-photorealistic-environments.md
│   │   │   ├── week-7-isaac-ros-hardware-accelerated/
│   │   │   │   ├── chapter-13-isaac-ros-vslam.md
│   │   │   │   └── chapter-14-nav2-integration.md
│   │   │   └── week-8-isaac-sim-reinforcement-learning/
│   │   │       ├── chapter-15-isaac-gym-gpu-rl.md
│   │   │       └── chapter-16-domain-randomization.md
│   │   └── module-4-vision-language-action/
│   │       ├── index.md
│   │       ├── week-9-voice-to-action-with-openai-whisper/
│   │       │   ├── chapter-17-whisper-speech-recognition.md
│   │       │   └── chapter-18-voice-ros2-integration.md
│   │       ├── week-10-cognitive-planning/
│   │       │   ├── chapter-19-llm-cognitive-planning.md
│   │       │   └── chapter-20-action-planning-safety.md
│   │       ├── week-11-system-integration/
│   │       │   ├── chapter-21-system-architecture.md
│   │       │   └── chapter-22-vision-multimodal.md
│   │       ├── week-12-advanced-deployment/
│   │       │   ├── chapter-23-real-world-deployment.md
│   │       │   └── chapter-24-learning-adaptation.md
│   │       └── week-13-testing-validation/
│   │           ├── chapter-25-testing-validation.md
│   │           └── chapter-26-final-deployment.md
│   ├── src/
│   │   ├── components/            # React components
│   │   │   └── RagChatbot/        # Floating chatbot widget
│   │   ├── services/api/          # API integration layer
│   │   ├── pages/                 # Custom pages
│   │   ├── theme/                 # Theme customization
│   │   │   ├── Layout.jsx         # Custom layout wrapper
│   │   │   └── DocItem/           # Custom doc item theme
│   │   ├── css/                   # Global styles
│   │   └── utils/                 # Utility functions
│   ├── static/                    # Static assets
│   ├── blog/                      # Blog posts
│   ├── docusaurus.config.ts       # Main configuration
│   ├── sidebars.ts                # Sidebar navigation
│   ├── package.json               # Dependencies
│   └── README.md                  # Frontend documentation
│
├── backend/                       # Backend (FastAPI)
│   ├── api.py                     # FastAPI application
│   ├── agent.py                   # RAG agent with LLM
│   ├── retrieving.py              # Vector retrieval
│   ├── main.py                    # Embedding pipeline
│   ├── requirements.txt           # Python dependencies
│   ├── Dockerfile                 # Container config
│   ├── schemas/                   # Pydantic schemas
│   ├── services/                  # Service modules
│   └── README.md                  # Backend documentation
│
├── specs/                         # Feature specifications
│   ├── 001-ai-textbook-physical-ai/
│   │   ├── spec.md                # Feature specification
│   │   ├── plan.md                # Implementation plan
│   │   ├── tasks.md               # Task breakdown
│   │   ├── research.md            # Research notes
│   │   ├── data-model.md          # Data models
│   │   ├── quickstart.md          # Quick start guide
│   │   ├── checklists/            # Requirement checklists
│   │   └── contracts/             # API contracts
│   ├── 002-rag-chatbot/
│   └── 004-rag-agent-frontend-integration/
│
├── history/                       # Project history
│   ├── adr/                       # Architecture Decision Records
│   └── prompts/                   # Prompt History Records
│       ├── constitution/          # Constitution-related prompts
│       ├─ general/               # General prompts
│       └── [feature-name]/        # Feature-specific prompts
│
├── .specify/                      # Spec-Kit Plus configuration
│   ├── memory/                    # Project memory
│   │   └── constitution.md        # Project principles
│   ├── templates/                 # Document templates
│   └── scripts/                   # Automation scripts
│
├── .claude/                       # Claude Code configuration
│   ├── commands/                  # Custom commands
│   └── settings.local.json        # Local settings
│
├── CLAUDE.md                      # Claude Code rules
├── Hackathon_1.md                 # Project requirements
├── .dockerignore                  # Docker ignore patterns
├── .env                           # Environment variables (not in git)
└── README.md                      # This file
```

---

## 🛠️ Technology Stack

### Frontend Stack

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

### Backend Stack

| Technology | Version | Purpose |
|------------|---------|---------|
| FastAPI | Latest | Web framework |
| Python | 3.11+ | Programming language |
| Uvicorn | Latest | ASGI server |
| Cohere | 4.9+ | Text embeddings |
| OpenRouter | Latest | LLM API gateway |
| Qdrant | 1.9+ | Vector database |
| BeautifulSoup4 | 4.12+ | HTML parsing |
| Pydantic | 2.0+ | Data validation |

### Infrastructure

| Service | Purpose |
|---------|---------|
| GitHub Pages | Frontend hosting |
| Hugging Face Spaces | Backend hosting |
| Qdrant Cloud | Vector database hosting |
| Cohere API | Embedding generation |
| OpenRouter | LLM inference |

---

## 🚀 Getting Started

### Prerequisites

**Frontend:**
- Node.js ≥ 20.0
- npm or yarn

**Backend:**
- Python 3.11+
- pip or uv

**API Keys Required:**
- Cohere API key (for embeddings)
- OpenRouter API key (for LLM)
- Qdrant Cloud URL and API key (for vector storage)

### Quick Start

Clone the repository and navigate to the project directory.

**Frontend Setup:**
1. Navigate to my-website directory
2. Install dependencies
3. Start development server

**Backend Setup:**
1. Navigate to backend directory
2. Install Python dependencies
3. Create .env file with required API keys
4. Start FastAPI server

**Important:** Never commit the .env file. It contains sensitive API keys.

---

## 🔄 Development Workflow

### Frontend Development

Navigate to my-website directory and start the development server. Make changes to docs/ for content, src/components/ for React components, and src/css/ for styles. Changes auto-reload in the browser.

### Backend Development

Navigate to backend directory and start the server with auto-reload. Make changes to api.py for endpoints, agent.py for RAG logic, and retrieving.py for vector search. Server auto-restarts on file changes.

### Embedding Pipeline

Before using the chatbot, run the embedding pipeline to ingest textbook content. This fetches all pages from the deployed site, extracts text, chunks it, generates embeddings, and stores them in Qdrant.

---

## 🚀 Deployment

### Frontend Deployment (GitHub Pages)

The frontend is deployed to GitHub Pages automatically. Configure the repository settings and use the deployment command. Requires push access to the repository.

**Configuration:**
- Repository: muhammadwaheedairi/hackathon_textbook_ai_robotics
- Branch: gh-pages
- Base URL: /hackathon_textbook_ai_robotics/

### Backend Deployment (Hugging Face Spaces)

The backend is deployed to Hugging Face Spaces using Docker.

**Steps:**
1. Create new Space with Docker SDK
2. Configure environment variables in Space settings
3. Push code to Space repository
4. Automatic deployment on push

**Environment Variables:**
Configure all required API keys in Space settings (never in code).

---

## 📡 API Documentation

### POST /ask

Submit a question and receive an AI-generated answer.

**Request Body:**
- query: string (required, max 2000 characters)

**Response:**
- answer: Generated answer text
- sources: List of source URLs
- matched_chunks: Retrieved content chunks with similarity scores
- status: success/error/empty
- query_time_ms: Processing time
- confidence: Confidence level

**Status Codes:**
- 200: Success
- 400: Invalid query
- 500: Internal server error

### GET /health

Health check endpoint returning service status.

---

## 📝 Content Management

### Adding New Chapters

1. Create Markdown file in appropriate module directory
2. Add frontmatter with sidebar_position and title
3. Update sidebars.ts to include new content
4. Rebuild and deploy frontend
5. Re-run embedding pipeline to index new content

### Markdown Features

- Syntax-highlighted code blocks
- Admonitions for notes, tips, and warnings
- Tabs for multi-language examples
- MDX support for React components in Markdown

---

## 📋 Spec-Driven Development

This project uses **Spec-Kit Plus** methodology for structured development.

### Specification Structure

Each feature has a dedicated directory under `specs/` containing:

- **spec.md** - Feature requirements and acceptance criteria
- **plan.md** - Implementation plan and architecture decisions
- **tasks.md** - Detailed task breakdown with dependencies
- **research.md** - Research notes and findings
- **data-model.md** - Data structures and schemas
- **quickstart.md** - Quick start guide for the feature
- **checklists/** - Requirement validation checklists
- **contracts/** - API contracts and interfaces

### History Tracking

**Architecture Decision Records (ADRs):**
Located in `history/adr/`, documenting significant architectural decisions with context, options considered, and rationale.

**Prompt History Records (PHRs):**
Located in `history/prompts/`, capturing all AI-assisted development interactions for traceability and learning.

### Project Memory

The `.specify/memory/constitution.md` file contains project principles, coding standards, and architectural guidelines that guide all development decisions.

---

## 🧪 Testing

### Frontend Testing

- Type checking with TypeScript compiler
- Build testing to verify production builds
- Manual testing of navigation and chatbot
- Accessibility testing with keyboard navigation
- Responsive design testing on multiple devices

### Backend Testing

- API endpoint testing with curl or testing tools
- Retrieval system testing with test queries
- Embedding pipeline testing with sample content
- Performance testing for response times
- Integration testing with frontend

### End-to-End Testing

1. Start both frontend and backend locally
2. Test chatbot functionality
3. Verify answer accuracy and sources
4. Check error handling
5. Test on multiple devices and browsers

---

## ⚡ Performance

### Frontend Performance

- Static Site Generation for instant loading
- Code splitting and lazy loading
- Asset optimization (minified CSS/JS, compressed images)
- CDN delivery via GitHub Pages

**Lighthouse Scores:**
- Performance: 95+
- Accessibility: 100
- Best Practices: 95+
- SEO: 100

### Backend Performance

**Typical Response Times:**
- Embedding generation: 200-500ms
- Vector search: 50-100ms
- LLM generation: 1000-3000ms
- Total query time: 1500-4000ms

**Optimization Strategies:**
- Async request handling
- Connection pooling
- Qdrant Cloud for fast vector search
- Efficient chunking strategy (1000 chars with 100 overlap)

---

## 🔐 Security

### Security Best Practices

**Environment Variables:**
- Store all API keys in .env files
- Never commit .env to version control
- Use different keys for development and production
- Regularly rotate API keys

**Frontend Security:**
- HTTPS only (enforced by GitHub Pages)
- No sensitive data in client code
- CORS requests to trusted backend only
- Content Security Policy headers

**Backend Security:**
- Environment variables for all secrets
- Input validation via Pydantic
- Rate limiting (recommended for production)
- CORS configured for specific origins
- No user data storage
- Regular security audits

**Production Recommendations:**
- Configure CORS for specific origins only
- Implement rate limiting on API endpoints
- Monitor API usage and set quotas
- Use HTTPS for all communications
- Implement request logging and monitoring

---

## 🚨 Troubleshooting

### Frontend Issues

**Build Failures:**
- Clear cache and reinstall dependencies
- Check Node.js version compatibility
- Verify all imports are correct

**Chatbot Not Connecting:**
- Check browser console for errors
- Verify API URL configuration
- Check backend service status
- Test backend health endpoint directly

**Styles Not Applying:**
- Clear Docusaurus cache
- Restart development server
- Check CSS syntax errors

### Backend Issues

**Connection Failures:**
- Verify all API keys are set correctly
- Check network connectivity
- Confirm services are running

**Empty Retrieval Results:**
- Verify embedding pipeline ran successfully
- Check collection exists in Qdrant
- Lower similarity threshold
- Verify content was properly indexed

**Performance Issues:**
- Monitor API response times
- Check Qdrant Cloud performance
- Optimize chunk size and overlap
- Implement caching layer

### Common Error Messages

**"Module not found":**
Clear cache and reinstall dependencies.

**"API key invalid":**
Verify API keys in .env file are correct and active.

**"Collection not found":**
Run the embedding pipeline to create and populate the collection.

**"CORS error":**
Check CORS configuration in backend API.

---

## 🤝 Contributing

### How to Contribute

1. Fork the repository
2. Create feature branch
3. Make changes following project conventions
4. Test thoroughly (frontend, backend, integration)
5. Update documentation if needed
6. Commit changes with clear messages
7. Push to branch
8. Open Pull Request with description

### Code Style

**Frontend:**
- Follow TypeScript best practices
- Use functional components with hooks
- Add proper TypeScript types
- Follow Docusaurus conventions
- Use CSS modules for component styles

**Backend:**
- Follow PEP 8 style guide
- Add type hints to all functions
- Write docstrings for classes and methods
- Use async/await for I/O operations
- Keep functions focused and small

### Commit Messages

Use clear, descriptive commit messages:
- feat: Add new feature
- fix: Bug fix
- docs: Documentation changes
- style: Code style changes
- refactor: Code refactoring
- test: Test additions or changes
- chore: Maintenance tasks

---

## 📄 License

This project is part of Hackathon I for Panaversity. All rights reserved.

---

## 🙏 Acknowledgments

- **Panaversity** - For organizing the hackathon and providing the opportunity
- **Docusaurus** - For the amazing documentation framework
- **FastAPI** - For the high-performance web framework
- **Cohere** - For powerful embedding models
- **Qdrant** - For efficient vector search capabilities
- **OpenRouter** - For LLM API access
- **Hugging Face** - For free backend hosting

---

## 📞 Contact

**Project Repository:** [GitHub](https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics)

**Live Textbook:** [https://muhammadwaheedairi.github.io/hackathon_textbook_ai_robotics/](https://muhammadwaheedairi.github.io/hackathon_textbook_ai_robotics/)

---

## 🎓 Learning Resources

### Robotics & AI
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac Documentation](https://developer.nvidia.com/isaac)
- [Gazebo Simulation](https://gazebosim.org/)

### Web Development
- [Docusaurus Documentation](https://docusaurus.io/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [React Documentation](https://react.dev/)
- [TypeScript Handbook](https://www.typescriptlang.org/docs/)

### AI & ML
- [Cohere Documentation](https://docs.cohere.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenRouter Documentation](https://openrouter.ai/docs)

---

**Built with ❤️ for the future of Physical AI & Humanoid Robotics education**
