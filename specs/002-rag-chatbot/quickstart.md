# Quickstart: RAG Chatbot Implementation

## Prerequisites

- Python 3.11+
- Docker and Docker Compose (for local development)
- OpenAI API key
- Cohere API key
- Qdrant Cloud account and API key
- Claude Context7 MCP server endpoint
- Git

## Local Development Setup

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd <repo-name>
   ```

2. **Set up Python environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r backend/requirements.txt
   pip install -r backend/requirements-dev.txt
   ```

3. **Configure environment variables**
   ```bash
   cp backend/.env.example backend/.env
   # Edit backend/.env with your actual API keys and configuration
   ```

4. **Start Qdrant locally (optional for development)**
   ```bash
   docker-compose up -d
   ```

## Running the Service

1. **Start the backend service**
   ```bash
   cd backend
   uvicorn app.main:app --reload --port 8000
   ```

2. **Access the API**
   - API documentation: http://localhost:8000/docs
   - Health check: http://localhost:8000/api/v1/health

## Key Endpoints

- `POST /api/v1/chat` - Main chat endpoint
- `GET /api/v1/chat/{session_id}` - Get session history
- `POST /api/v1/content/index` - Index book content
- `GET /api/v1/health` - Health check

## Testing

1. **Run unit tests**
   ```bash
   cd backend
   pytest tests/unit/
   ```

2. **Run integration tests**
   ```bash
   cd backend
   pytest tests/integration/
   ```

## Configuration

Key configuration parameters in `backend/.env`:

- `OPENAI_API_KEY` - Your OpenAI API key
- `COHERE_API_KEY` - Your Cohere API key
- `MCP_CONTEXT7_ENDPOINT` - Claude Context7 MCP server endpoint
- `QDRANT_URL` - Qdrant Cloud URL or local address
- `QDRANT_API_KEY` - Qdrant API key (if using cloud)
- `QDRANT_COLLECTION_NAME` - Name of the vector collection to use
- `RATE_LIMIT_REQUESTS` - Number of requests per minute (default: 60)
- `RATE_LIMIT_WINDOW` - Time window in seconds (default: 60)
- `SESSION_TIMEOUT_MINUTES` - Session timeout in minutes (default: 30)