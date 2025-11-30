"""
Chatbot API endpoints
"""
from fastapi import APIRouter, HTTPException
from ..models.schemas import ChatRequest, ChatResponse
from ..services.chatbot_service import chatbot_service
from ..services.qdrant_service import qdrant_service
from ..services.db_service import db_service

router = APIRouter()

@router.post("/ask", response_model=ChatResponse)
async def ask_question(request: ChatRequest):
    """
    Process a question and return an answer with citations
    """
    try:
        response = await chatbot_service.process_question(request)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/health")
async def health():
    """Check service health"""
    qdrant_healthy = qdrant_service.health_check()
    db_healthy = db_service.health_check()

    return {
        "status": "healthy" if (qdrant_healthy and db_healthy) else "unhealthy",
        "qdrant": "connected" if qdrant_healthy else "disconnected",
        "database": "connected" if db_healthy else "disconnected"
    }
