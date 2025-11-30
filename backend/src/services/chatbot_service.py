"""
Main chatbot service orchestrating RAG pipeline
"""
import uuid
from typing import List, Dict, Optional
from .qdrant_service import qdrant_service
from .embedding_service import embedding_service
from .llm_service import llm_service
from .db_service import db_service
from ..models.schemas import ChatRequest, ChatResponse, Citation

class ChatbotService:
    def __init__(self):
        self.qdrant = qdrant_service
        self.embeddings = embedding_service
        self.llm = llm_service
        self.db = db_service

    async def process_question(self, request: ChatRequest) -> ChatResponse:
        """Process a question and generate an answer with citations"""
        try:
            # Generate session ID if not provided
            session_id = request.session_id or str(uuid.uuid4())

            # Get or create conversation
            conversation_id = self.db.get_or_create_conversation(session_id)

            # Generate embedding for the question
            question_embedding = self.embeddings.generate_embedding(request.question)

            # Search for relevant chunks
            results = self.qdrant.search(
                query_vector=question_embedding,
                limit=5,
                current_page=request.current_page
            )

            # Get conversation history
            conversation_history = self.db.get_conversation_history(session_id, limit=6)

            # Generate answer
            answer = self.llm.generate_answer(
                question=request.question,
                context_chunks=results,
                conversation_history=conversation_history,
                selected_text=request.selected_text
            )

            # Extract citations from results
            citations = self._create_citations(results)

            # Save messages to database
            self.db.save_message(conversation_id, "user", request.question)
            self.db.save_message(conversation_id, "assistant", answer)

            return ChatResponse(
                answer=answer,
                citations=citations,
                session_id=session_id
            )

        except Exception as e:
            print(f"Error processing question: {e}")
            return ChatResponse(
                answer="I'm sorry, I encountered an error processing your question. Please try again.",
                citations=[],
                session_id=request.session_id or str(uuid.uuid4()),
                error=str(e)
            )

    def _create_citations(self, search_results: List[Dict]) -> List[Citation]:
        """Create citation objects from search results"""
        citations = []
        seen_sections = set()

        for result in search_results:
            metadata = result.get("metadata", {})
            section = metadata.get("section", "Unknown Section")

            # Avoid duplicate citations
            if section in seen_sections:
                continue

            seen_sections.add(section)

            citation = Citation(
                title=metadata.get("title", "Unknown"),
                url=metadata.get("page_url", "#"),
                section=section,
                snippet=result.get("content", "")[:200] + "..."
            )
            citations.append(citation)

        return citations

# Global instance
chatbot_service = ChatbotService()
