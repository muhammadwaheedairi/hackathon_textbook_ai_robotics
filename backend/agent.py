import os
import json
import logging
import time
from typing import Dict
from dotenv import load_dotenv
from agents import Agent, Runner, function_tool
from retrieving import RAGRetriever

load_dotenv()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Minimum similarity score to trust a retrieved chunk.
MIN_SCORE = 0.3


def _tool_failure_message(e: Exception) -> str:
    """Called by the Agents SDK if search_textbook raises an exception,
    instead of crashing the whole agent run."""
    logger.error(f"search_textbook tool failed: {e}")
    return "NO_RELEVANT_CONTENT_FOUND"


@function_tool(failure_error_function=_tool_failure_message)
def search_textbook(query: str) -> str:
    """Search the 'Physical AI & Humanoid Robotics' textbook for passages relevant
    to the given query. Always call this before answering any question.

    Args:
        query: The user's question, or a short search phrase describing what to look up.

    Returns:
        Matching passages from the textbook with their source URLs, or the literal
        string "NO_RELEVANT_CONTENT_FOUND" if nothing relevant exists in the textbook.
    """
    retriever = RAGRetriever()
    raw = retriever.retrieve(query, top_k=5)
    data = json.loads(raw)
    chunks = data.get("results", [])

    good_chunks = [
        c for c in chunks
        if c.get("content") and c.get("similarity_score", 0) >= MIN_SCORE
    ]

    if not good_chunks:
        logger.warning(f"No relevant chunks found for query: {query[:50]}")
        return "NO_RELEVANT_CONTENT_FOUND"

    formatted = ""
    for c in good_chunks:
        formatted += f"Source: {c['url']}\n{c['content']}\n\n"
    return formatted


# The agent: an LLM configured with instructions + the retrieval tool.
# It decides on its own to call search_textbook, then answers from the result.
textbook_agent = Agent(
    name="Textbook Assistant",
    instructions=(
        "You are a helpful assistant for the 'Physical AI & Humanoid Robotics' textbook. "
        "For every question, you MUST call the search_textbook tool first - never answer "
        "before calling it, even if you think you already know the answer. "
        "Only answer using information returned by the tool. Never use outside or general "
        "knowledge, even if it's true. "
        "If the tool returns NO_RELEVANT_CONTENT_FOUND, or what it returns isn't actually "
        "relevant to the question, tell the user this topic isn't covered in the textbook - "
        "do not guess. "
        "When the tool does return relevant passages, synthesize a clear, concise answer "
        "from them, combining details across passages if needed. Keep answers focused and "
        "avoid unnecessary repetition."
    ),
    tools=[search_textbook],
    model="gpt-4o-mini",
)


class RAGAgent:
    def __init__(self):
        logger.info("RAG Agent initialized with OpenAI Agents SDK (gpt-4o-mini)")

    async def query_agent(self, query_text: str) -> Dict:
        """Async entrypoint - use this from FastAPI (await rag_agent.query_agent(...))."""
        start_time = time.time()

        try:
            result = await Runner.run(textbook_agent, query_text)
            answer = (result.final_output or "").strip()

            return {
                "answer": answer,
                "query_time_ms": (time.time() - start_time) * 1000
            }

        except Exception as e:
            logger.error(f"Error in RAGAgent query_agent: {e}")
            return {
                "answer": "",
                "error": str(e),
                "query_time_ms": (time.time() - start_time) * 1000
            }

    def query_agent_sync(self, query_text: str) -> Dict:
        """Sync wrapper, kept for any non-async callers (e.g. CLI scripts)."""
        start_time = time.time()

        try:
            result = Runner.run_sync(textbook_agent, query_text)
            answer = (result.final_output or "").strip()

            return {
                "answer": answer,
                "query_time_ms": (time.time() - start_time) * 1000
            }

        except Exception as e:
            logger.error(f"Error in RAGAgent query_agent_sync: {e}")
            return {
                "answer": "",
                "error": str(e),
                "query_time_ms": (time.time() - start_time) * 1000
            }