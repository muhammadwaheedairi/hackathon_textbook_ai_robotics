import os
import asyncio
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the existing RAG agent functionality
from agent import RAGAgent

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG Agent with document retrieval and question answering",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    answer: str
    error: Optional[str] = None
    status: str  # "success", "error"

class HealthResponse(BaseModel):
    status: str
    message: str

# Global RAG agent instance
rag_agent = None

@app.on_event("startup")
async def startup_event():
    """Initialize the RAG agent on startup"""
    global rag_agent
    logger.info("Initializing RAG Agent...")
    try:
        rag_agent = RAGAgent()
        logger.info("RAG Agent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}")
        raise

@app.post("/ask", response_model=QueryResponse)
async def ask_rag(request: QueryRequest):
    """Process a user query and return only the answer"""
    logger.info(f"Processing query: {request.query[:50]}...")

    try:
        # Validate input
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(request.query) > 2000:
            raise HTTPException(status_code=400, detail="Query too long, maximum 2000 characters")

        # Process query through RAG agent
        response = rag_agent.query_agent(request.query)

        # Return only answer
        return QueryResponse(
            answer=response.get("answer", ""),
            error=response.get("error"),
            status="error" if response.get("error") else "success"
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        return QueryResponse(
            answer="",
            error=str(e),
            status="error"
        )

@app.get("/health", response_model=HealthResponse)
async def health_check():
    return HealthResponse(
        status="healthy",
        message="RAG Agent API is running"
    )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)