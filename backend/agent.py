import os
import json
import logging
import time
from typing import Dict
from dotenv import load_dotenv
from openai import OpenAI
from retrieving import RAGRetriever

load_dotenv()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGAgent:
    def __init__(self):
        self.client = OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=os.getenv("OPENROUTER_API_KEY")
        )
        # Working free model
        self.model = "arcee-ai/trinity-large-preview:free"
        logger.info("RAG Agent initialized with OpenRouter free model")

    def query_agent(self, query_text: str) -> Dict:
        start_time = time.time()

        try:
            # Step 1: fetch chunks
            retriever = RAGRetriever()
            raw = retriever.retrieve(query_text, top_k=5)
            data = json.loads(raw)
            chunks = data.get("results", [])

            # Prepare context
            context = ""
            for c in chunks:
                context += f"{c['content']}\n\n"

            prompt = (
                "Answer the question concisely using the following context. "
                "If not in the context, answer based on general knowledge.\n\n"
                f"Context:\n{context}"
                f"Question: {query_text}\nAnswer:"
            )

            # Step 2: OpenRouter chat completion
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for a robotics textbook. Give concise, clear answers."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.5,
                max_tokens=256
            )

            answer = response.choices[0].message.content.strip()

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