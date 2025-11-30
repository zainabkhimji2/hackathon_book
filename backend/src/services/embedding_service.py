"""
OpenAI embedding generation service
"""
import os
from typing import List
from openai import OpenAI

class EmbeddingService:
    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-3-small")

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        response = self.client.embeddings.create(
            model=self.model,
            input=text
        )
        return response.data[0].embedding

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts"""
        response = self.client.embeddings.create(
            model=self.model,
            input=texts
        )
        return [item.embedding for item in response.data]

# Global instance
embedding_service = EmbeddingService()
