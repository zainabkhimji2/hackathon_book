"""
Qdrant vector database service
"""
import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue

class QdrantService:
    def __init__(self):
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_content")
        self.client = None

    def connect(self):
        """Initialize Qdrant client"""
        if self.url and self.api_key:
            self.client = QdrantClient(url=self.url, api_key=self.api_key)
        else:
            # Use local Qdrant for development
            self.client = QdrantClient(host="localhost", port=6333)
        return self.client

    def create_collection(self, vector_size: int = 1536):
        """Create collection if it doesn't exist"""
        if not self.client:
            self.connect()

        collections = self.client.get_collections().collections
        if not any(col.name == self.collection_name for col in collections):
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
            )
            print(f"Collection '{self.collection_name}' created")
        else:
            print(f"Collection '{self.collection_name}' already exists")

    def upsert_points(self, points: List[PointStruct]):
        """Insert or update points in collection"""
        if not self.client:
            self.connect()

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        current_page: Optional[str] = None
    ) -> List[Dict]:
        """Search for similar vectors"""
        if not self.client:
            self.connect()

        # Simple search without page filter (index not created)
        # TODO: Add page_url indexing in create_collection for page-specific filtering
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=limit,
            with_payload=True
        ).points

        return [
            {
                "id": hit.id,
                "score": hit.score,
                "content": hit.payload.get("content", ""),
                "metadata": hit.payload
            }
            for hit in results
        ]

    def health_check(self) -> bool:
        """Check if Qdrant is accessible"""
        try:
            if not self.client:
                self.connect()
            collections = self.client.get_collections()
            return True
        except Exception as e:
            print(f"Qdrant health check failed: {e}")
            return False

# Global instance
qdrant_service = QdrantService()
