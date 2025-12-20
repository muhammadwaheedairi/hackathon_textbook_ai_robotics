import os
import json
import logging
import time
from typing import Dict, List
from dotenv import load_dotenv
from openai import OpenAI  # OpenAI-compatible client for OpenRouter
from retrieving import RAGRetriever
from agents import function_tool

load_dotenv()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@function_tool
def retrieve_information(query: str) -> Dict:
    retriever = RAGRetriever()
    try:
        json_resp = retriever.retrieve(query_text=query, top_k=5, threshold=0.3)
        results = json.loads(json_resp).get("results", [])
        formatted = [
            {
                "content": r["content"],
                "url": r["url"],
                "position": r.get("position", 0),
                "similarity_score": r["similarity_score"]
            }
            for r in results
        ]
        return {
            "query": query,
            "retrieved_chunks": formatted,
            "total_results": len(formatted)
        }
    except Exception as e:
        logger.error(f"Error in retrieve_information: {e}")
        return {
            "query": query,
            "retrieved_chunks": [],
            "total_results": 0,
            "error": str(e)
        }

class RAGAgent:
    def __init__(self):
        self.client = OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=os.getenv("OPENROUTER_API_KEY")
        )
        # Free model
        self.model = "mistralai/devstral-2512:free"
        logger.info("RAG Agent initialized with OpenRouter free model")

    def query_agent(self, query_text: str) -> Dict:
        start_time = time.time()

        try:
            # Step 1: fetch chunks
            retriever = RAGRetriever()
            raw = retriever.retrieve(query_text, top_k=5)
            data = json.loads(raw)
            chunks = data.get("results", [])

            # Prepare prompt
            context = ""
            for c in chunks:
                context += f"URL: {c['url']}\n{c['content']}\n\n"

            prompt = (
                "Answer the question using the following retrieved context. "
                "If not in the context, answer based on general knowledge.\n\n"
                f"{context}"
                f"Question: {query_text}\nAnswer:"
            )

            # Step 2: OpenRouter chat completion
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.5,
                max_tokens=512
            )

            answer = response.choices[0].message.content.strip()

            query_time_ms = (time.time() - start_time) * 1000
            return {
                "answer": answer,
                "sources": [c["url"] for c in chunks],
                "matched_chunks": chunks,
                "query_time_ms": query_time_ms,
                "confidence": "medium"
            }

        except Exception as e:
            logger.error(f"Error in RAGAgent query_agent: {e}")
            return {
                "answer": "",
                "sources": [],
                "matched_chunks": [],
                "error": str(e),
                "query_time_ms": (time.time() - start_time) * 1000
            }
