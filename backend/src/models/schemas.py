"""
Pydantic models for API requests and responses
"""
from pydantic import BaseModel, Field
from typing import List, Optional

class ChatRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000, description="User's question")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Selected text context")
    current_page: Optional[str] = Field(None, description="Current page URL or path")
    session_id: Optional[str] = Field(None, description="Session ID for conversation history")

class Citation(BaseModel):
    title: str = Field(..., description="Section or page title")
    url: str = Field(..., description="Link to the textbook section")
    section: str = Field(..., description="Section name")
    snippet: Optional[str] = Field(None, description="Relevant text snippet")

class ChatResponse(BaseModel):
    answer: str = Field(..., description="Generated answer")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    session_id: str = Field(..., description="Session ID")
    error: Optional[str] = Field(None, description="Error message if any")

class ContentChunk(BaseModel):
    id: str
    content: str
    metadata: dict
    embedding: Optional[List[float]] = None
