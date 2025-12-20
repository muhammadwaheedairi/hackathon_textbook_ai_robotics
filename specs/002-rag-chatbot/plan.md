# Implementation Plan: RAG Chatbot for Book Content

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-16 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

## Summary

Implementation of a minimal Retrieval-Augmented Generation (RAG) chatbot for book content using simple Python scripts with Qdrant vector database and Cohere embeddings. The system will allow users to ask questions about book content and receive AI-generated answers based on semantic search of the book's content. The implementation includes a Docusaurus embedding pipeline that extracts content from sitemap, chunks and embeds text, stores in Qdrant, and provides retrieval functionality with a custom agent implementation.

## Technical Context

**Language/Version**: Python 3.11+ (required for async operations)
**Primary Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv
**Storage**: Qdrant Cloud (vector database for embeddings)
**Testing**: pytest for testing
**Target Platform**: Linux server (Python scripts)
**Project Type**: Simple RAG system with embedding pipeline and retrieval
**Performance Goals**: Reasonable response time for retrieval and generation for educational use
**Constraints**: Memory efficient chunking, respect API quotas for Cohere and Qdrant
**Scale/Scope**: Educational-focused implementation, single-user interaction

## RAG Implementation Flow

The RAG system follows a simple flow:

1. **Sitemap Parsing**: Extract content via /sitemap.xml to identify all book content pages
2. **Text Extraction**: Extract text content from each page identified in the sitemap
3. **Chunking**: Split content into chunks using text splitting with overlap to preserve context
4. **Embeddings**: Generate embeddings using Cohere's embedding models
5. **Qdrant Storage**: Store embeddings and metadata in Qdrant Cloud vector database
6. **Retrieval**: Perform semantic search to retrieve relevant content chunks based on user queries
7. **Agent Answering**: Use custom agent implementation to generate answers from retrieved content

## Educational Implementation Features

### Simple Architecture
- **Python Scripts**: Minimal implementation using simple Python scripts without complex frameworks
- **Direct Implementation**: No dependency injection or complex architecture patterns
- **Error Handling**: Basic error handling with logging for debugging purposes
- **Configuration**: Environment variable loading for API keys and settings

### RAG Pipeline Components
- **DocusaurusEmbeddingPipeline**: Class that handles the complete pipeline from sitemap to Qdrant storage
- **RAGRetriever**: Class that handles retrieval of relevant content based on user queries
- **RAGAgent**: Custom agent implementation that uses function tools to retrieve and answer

### Source Tracking
- **Similarity Scores**: Include similarity scores (0.0-1.0) for retrieved content chunks
- **Metadata**: Basic metadata including URL, position, and content of retrieved chunks
- **Source Attribution**: Clear source attribution in AI responses with URLs of source content

### Error Handling
- **Graceful Degradation**: Proper error handling when external services (Cohere, Qdrant) fail
- **Error Logging**: Basic logging for debugging and monitoring of the embedding and retrieval processes
- **User-Friendly Messages**: Clear error messages when operations fail

## Testing Strategy

### Unit Testing
- **Pipeline Components**: Unit tests for DocusaurusEmbeddingPipeline functionality (sitemap parsing, text extraction, chunking)
- **Retrieval Functions**: Unit tests for RAGRetriever methods (embedding generation, Qdrant queries)
- **Agent Functions**: Unit tests for RAGAgent functionality and response formatting

### Integration Testing
- **End-to-End RAG Flow**: Tests that verify the entire RAG flow from query to response
- **External API Integration**: Tests for Cohere and Qdrant API interactions
- **Database Integration**: Tests for Qdrant operations and content retrieval

### Testing Approach
- **Functional Tests**: Focus on testing the core functionality of embedding, retrieval, and response generation
- **Error Handling Tests**: Tests for handling of API failures and edge cases
- **Basic Test Coverage**: Reasonable test coverage for critical functionality

## Custom Agent Implementation

### Agent Design Requirements
- **Function Tool Approach**: Use function tools to enable the agent to retrieve information from the knowledge base
- **Content-First Approach**: Agent must retrieve content first before generating any response
- **Simple Architecture**: Use straightforward agent implementation without complex SDK dependencies
- **Direct Tool Integration**: Integrate retrieval functionality directly as a function tool

### Content-First Implementation
- **Retrieval First**: Agent must attempt content retrieval before generating any response
- **No Hallucination**: If no relevant content is found, respond appropriately rather than generating from general knowledge
- **Source Verification**: Verify that retrieved content is relevant before using it to generate responses

## Free-Tier Technology Emphasis

### Cohere for Embeddings
- **Free Tier Usage**: Utilize Cohere's free tier for embedding generation (up to 1M tokens/month)
- **Embedding Quality**: Use Cohere's embed-english-v2.0 model for high-quality English embeddings
- **Cost Management**: Implement proper token counting and usage monitoring to stay within free tier limits
- **Fallback Strategy**: Plan for potential migration to alternative embedding providers if usage exceeds free tier

### Qdrant Cloud for Vector Database
- **Free Tier Utilization**: Use Qdrant Cloud's free tier for vector storage and semantic search
- **Vector Management**: Efficient vector storage and retrieval to maximize free tier capacity
- **Performance Optimization**: Optimize queries to minimize resource usage while maintaining performance
- **Scalability Planning**: Design with migration path in mind if free tier limits are exceeded

### Cost-Effective Architecture
- **API Usage Monitoring**: Implement monitoring for all external API usage (OpenAI, Cohere, Qdrant)
- **Resource Optimization**: Optimize embedding generation and storage to minimize costs
- **Avoid Paid APIs**: Avoid using paid APIs where possible, focusing on free tier solutions
- **Usage Analytics**: Track usage patterns to predict and manage costs effectively

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Check

- **Textbook Creation Focus**: ✅ The RAG chatbot directly supports textbook content by providing AI-powered Q&A functionality based on book content
- **Curriculum Structure Adherence**: ✅ The feature integrates with the existing book structure without modifying curriculum content
- **Technical Stack Compliance**: ✅ Using Python with Cohere and Qdrant (aligns with specified Python requirements), integrates with existing Docusaurus platform
- **Technical Accuracy Requirement**: ✅ The RAG system will provide answers grounded in the book content with source information
- **Content Modularity**: ✅ The system works with modular book content chunks
- **Content Completeness**: ✅ The chatbot functionality will be available for all book content modules

### Constraints Compliance

- **Platform constraint**: ✅ Python scripts will integrate with existing Docusaurus structure via JavaScript integration
- **Tool constraint**: ✅ Development will use Claude Code + SpecifyPlus as specified
- **Structure constraint**: ✅ No changes to curriculum structure, only adding functionality
- **Deployment constraint**: ✅ Python scripts will be deployable separately, frontend integration will work with GitHub Pages

### Post-Design Verification

- **Architecture alignment**: ✅ Simple Python implementation with Qdrant vector database aligns with Python/ML ecosystem
- **Integration approach**: ✅ Script-based approach preserves existing book interface
- **Performance targets**: ✅ Response time target is achievable with proper implementation
- **Scalability**: ✅ Designed for educational use with single-user interaction

### Gate Status: PASSED

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file
├── spec.md              # Feature specification
└── tasks.md             # Implementation tasks
```

### Source Code (repository root)

```text
backend/
├── main.py              # Docusaurus embedding pipeline (sitemap parsing, text extraction, chunking, embedding, storage)
├── retrieving.py        # RAG retrieval functionality (query processing, Qdrant search, response formatting)
├── agent.py             # Custom agent implementation (function tools, response generation)
├── requirements.txt     # Python dependencies
├── .env.example         # Environment variables example
└── README.md            # Backend service documentation
```

**Structure Decision**: Selected minimal script-based structure with simple Python files. The implementation focuses on core RAG functionality without complex frameworks, providing a straightforward educational implementation.

## Complexity Tracking

No significant complexity violations identified. The implementation follows a minimal, educational approach that aligns with the intended scope.
