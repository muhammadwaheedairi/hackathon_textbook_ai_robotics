# 🤖 RAG Chatbot Backend

> FastAPI-powered RAG (Retrieval-Augmented Generation) backend for AI-native textbook Q&A

[![FastAPI](https://img.shields.io/badge/FastAPI-Latest-green.svg)](https://fastapi.tiangolo.com/)
[![Python](https://img.shields.io/badge/Python-3.11+-blue.svg)](https://www.python.org/)
[![Qdrant](https://img.shields.io/badge/Qdrant-1.9+-orange.svg)](https://qdrant.tech/)
[![Cohere](https://img.shields.io/badge/Cohere-4.9+-purple.svg)](https://cohere.com/)

---

## 🎯 Overview

High-performance RAG backend that retrieves relevant textbook content and generates contextual answers using vector similarity search and LLM-powered generation.

### Key Features

- **🔍 Vector Search** - Semantic search using Cohere embeddings and Qdrant
- **🧠 LLM Generation** - Answer generation via OpenRouter
- **⚡ Fast API** - Async FastAPI with CORS support
- **📊 Metadata Tracking** - Query time, confidence scores, and source URLs
- **🔄 Auto-Embedding** - Automated pipeline for textbook content ingestion
- **🐳 Docker Ready** - Containerized deployment

---

## 📁 Project Structure

```
backend/
├── api.py                  # FastAPI application and endpoints
├── agent.py                # RAG agent with LLM integration
├── retrieving.py           # Vector retrieval from Qdrant
├── main.py                 # Embedding pipeline for content ingestion
├── requirements.txt        # Python dependencies
├── Dockerfile              # Container configuration
├── pyproject.toml          # Project metadata
├── sdk.md                  # SDK documentation
├── schemas/                # Pydantic schemas
└── services/               # Service modules
```

---

## 🛠️ Tech Stack

### Core Framework
- **FastAPI** - Modern async web framework
- **Uvicorn** - ASGI server
- **Pydantic** - Data validation

### AI/ML Stack
- **Cohere** - Text embeddings (embed-multilingual-v3.0)
- **OpenRouter** - LLM API gateway
- **Qdrant** - Vector database for similarity search

### Data Processing
- **BeautifulSoup4** - HTML parsing
- **Requests** - HTTP client
- **python-dotenv** - Environment management

---

## 🚀 Getting Started

### Prerequisites

- Python 3.11 or higher
- pip or uv package manager

### Environment Variables

Create a `.env` file with required API keys:
- COHERE_API_KEY
- OPENROUTER_API_KEY
- QDRANT_URL
- QDRANT_API_KEY

**Important:** Never commit the .env file to version control. Add it to .gitignore.

### Installation

Navigate to the backend directory and install dependencies using pip or uv.

### Running Locally

Start the FastAPI server using uvicorn. The API will be available at localhost:8000 with interactive documentation at /docs endpoint.

### Docker Deployment

Build the Docker image and run the container with environment variables. The application exposes port 7860 for Hugging Face Spaces compatibility.

---

## 📡 API Endpoints

### POST /ask
Submit a question and receive an AI-generated answer with sources and metadata.

**Request Body:**
- query: string (required, max 2000 characters)

**Response Fields:**
- answer: Generated answer text
- sources: List of source URLs
- matched_chunks: Retrieved content chunks with similarity scores
- status: success/error/empty
- query_time_ms: Processing time in milliseconds
- confidence: Confidence level (low/medium/high)

**Status Codes:**
- 200: Success
- 400: Invalid query (empty or exceeds length limit)
- 500: Internal server error

### GET /health
Health check endpoint returning service status and availability.

---

## 🔧 Core Components

### 1. RAG Agent (agent.py)

Orchestrates the retrieval and generation pipeline. Initializes LLM client, retrieves relevant chunks, builds context-aware prompts, and generates answers with source attribution.

**Key Responsibilities:**
- Query processing and validation
- Context building from retrieved chunks
- LLM interaction and response formatting
- Performance metrics tracking

### 2. Retriever (retrieving.py)

Handles vector similarity search in Qdrant database.

**Key Responsibilities:**
- Generate query embeddings via Cohere
- Perform semantic search using cosine similarity
- Filter results by similarity threshold
- Preserve metadata (URL, position, timestamp)

### 3. Embedding Pipeline (main.py)

Automated content ingestion from deployed Docusaurus site.

**Pipeline Flow:**
1. Fetch sitemap from Docusaurus site
2. Extract and clean text from each page
3. Chunk text (1000 chars with 100 char overlap)
4. Generate embeddings via Cohere
5. Store in Qdrant with metadata

**Running Pipeline:**
Execute the main.py script to ingest content from the live textbook site.

### 4. FastAPI Application (api.py)

REST API with CORS support, async request handling, Pydantic validation, error handling, and logging.

---

## 🔍 Vector Database Schema

### Qdrant Collection: rag_embedding

**Vector Configuration:**
- Dimension: 1024 (Cohere embed-multilingual-v3.0)
- Distance Metric: Cosine similarity

**Payload Fields:**
- content: Text chunk content
- url: Source URL
- position: Chunk position in document
- created_at: Timestamp

---

## 🧪 Testing

### API Testing

Test health and ask endpoints using curl or API testing tools. Verify response format and status codes.

### Retrieval Testing

Run the retrieving.py script to test vector search functionality and view stored data.

### Embedding Pipeline Testing

Execute main.py to test the complete content ingestion pipeline.

---

## 📊 Performance Metrics

### Typical Response Times
- Embedding generation: 200-500ms
- Vector search: 50-100ms
- LLM generation: 1000-3000ms
- Total query time: 1500-4000ms

### Optimization Strategies
- Use Qdrant Cloud for faster vector search
- Implement caching for frequent queries
- Batch embedding generation
- Use streaming for LLM responses
- Optimize chunk size and overlap

---

## 🐳 Docker Configuration

### Container Setup

The Dockerfile uses Python 3.11 slim image, installs dependencies, and exposes port 7860 for Hugging Face Spaces deployment.

### Deployment Platforms

**Hugging Face Spaces:**
1. Create new Space with Docker SDK
2. Configure environment variables in Space settings
3. Push code to Space repository
4. Automatic deployment on push

**Other Platforms:**
Compatible with any Docker-supporting platform (AWS, GCP, Azure, etc.)

---

## 🔐 Security Best Practices

- Store all API keys in environment variables
- Never commit sensitive data to version control
- Use HTTPS in production
- Implement rate limiting for API endpoints
- Validate and sanitize all user input
- Configure CORS for specific origins in production
- Monitor API usage and set quotas
- Regularly rotate API keys

---

## 🚨 Troubleshooting

### Connection Issues

**Qdrant:**
- Verify URL and API key configuration
- Check network connectivity
- Confirm collection exists

**Cohere API:**
- Verify API key validity
- Check rate limits and quotas
- Ensure text length within limits

**OpenRouter:**
- Verify API key
- Check model availability
- Monitor usage quotas

### Empty Results

- Verify embedding pipeline completed successfully
- Check collection exists in Qdrant
- Lower similarity threshold
- Verify content was properly indexed

### Performance Issues

- Monitor API response times
- Check Qdrant Cloud performance
- Optimize chunk size and overlap
- Implement caching layer

---

## 📚 Dependencies

Core dependencies include FastAPI, Uvicorn, Cohere, Qdrant-client, BeautifulSoup4, Requests, OpenAI SDK, Pydantic, and python-dotenv. See requirements.txt for complete list with versions.

---

## 🔄 CI/CD

### Hugging Face Spaces

Automatic deployment pipeline:
- Push to main branch triggers rebuild
- Environment variables managed in Space settings
- Build logs available in dashboard
- Zero-downtime deployments

---

## 📈 Future Enhancements

- Response caching for improved performance
- Conversation history support
- Multi-language query support
- Streaming responses for better UX
- Analytics and monitoring dashboard
- Advanced reranking algorithms
- Query optimization and preprocessing

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Test changes thoroughly
4. Submit pull request with description

---

## 📄 License

Part of Hackathon I project for Panaversity.

---

**Powered by FastAPI, Cohere, Qdrant, and OpenRouter**
