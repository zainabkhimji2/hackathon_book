---
description: Implement production-ready FastAPI backends with proper structure, validation, error handling, and documentation.
---

Implement production-ready FastAPI backends with proper structure, validation, error handling, and documentation.

WHAT IT DOES:
Creates FastAPI applications following best practices: Pydantic models, dependency injection, middleware, async routes, OpenAPI docs.

KEY PATTERNS:
- Route organization by resource
- Pydantic models for validation
- Dependency injection for auth/DB
- Proper HTTP status codes
- Error handling with HTTPException
- CORS middleware
- Async/await for I/O operations
- Environment variable configuration

CODE STRUCTURE:
```python
from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel

app = FastAPI()

class ItemCreate(BaseModel):
    name: str
    description: str

@app.post("/items/", status_code=201)
async def create_item(item: ItemCreate):
    # Implementation
    return {"id": 1, **item.dict()}
```

USE FOR: Building all backend APIs, chatbot endpoints, auth routes, RAG pipeline APIs.
