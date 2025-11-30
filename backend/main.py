"""
FastAPI backend for RAG-powered chatbot
"""
import os
from pathlib import Path
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables from .env in backend directory
env_path = Path(__file__).parent / '.env'
load_dotenv(dotenv_path=env_path)

from src.api import chatbot

# Create FastAPI app
app = FastAPI(
    title="Physical AI Chatbot API",
    description="RAG-powered chatbot for Physical AI textbook",
    version="1.0.0"
)

# Configure CORS
origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chatbot.router, prefix="/api", tags=["chatbot"])

@app.get("/")
async def root():
    return {"message": "Physical AI Chatbot API", "status": "running"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("BACKEND_PORT", 8000))
    host = os.getenv("BACKEND_HOST", "0.0.0.0")
    uvicorn.run("main:app", host=host, port=port, reload=True)
