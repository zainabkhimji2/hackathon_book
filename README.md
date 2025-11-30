# RAG-Powered AI Chatbot for Physical AI Textbook

An intelligent AI chatbot that helps students learn Physical AI and robotics by answering questions about textbook content with accurate citations.

## Features

- 💬 Ask questions and get cited answers from the textbook
- 📖 Click citations to navigate to source sections
- 🎯 Highlight text and ask specific questions
- 💭 Multi-turn conversations with context
- 🌓 Dark/light mode support
- 📱 Mobile responsive

## Quick Setup

### 1. Environment Setup

Copy `.env.example` to `.env` and add your API keys:

```bash
cp .env.example .env
```

Required keys:
- `OPENAI_API_KEY` - Your OpenAI API key
- `QDRANT_URL` - Your Qdrant Cloud URL (or use local)
- `QDRANT_API_KEY` - Your Qdrant API key
- `DATABASE_URL` - Your Neon Postgres connection string

### 2. Install Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 3. Setup Database

Run this to create tables:

```bash
python -c "from src.services.db_service import db_service; db_service.create_tables(); print('Tables created')"
```

### 4. Ingest Textbook Content

```bash
cd backend
python scripts/ingest_content.py
```

This will:
- Process markdown files from `physical-ai-textbook/docs/`
- Generate embeddings
- Store in Qdrant

### 5. Start Backend Server

```bash
cd backend
python main.py
```

Backend runs at: http://localhost:8000

### 6. Integrate Chat Widget into Docusaurus

Add to `physical-ai-textbook/src/theme/Root.js`:

```jsx
import React from 'react';
import ChatWidget from '../components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

If `Root.js` doesn't exist, create it.

### 7. Add API URL to Docusaurus

In `physical-ai-textbook/docusaurus.config.js`, add:

```js
customFields: {
  REACT_APP_API_URL: 'http://localhost:8000/api',
},
```

### 8. Start Docusaurus

```bash
cd physical-ai-textbook
npm start
```

## Project Structure

```
├── backend/
│   ├── main.py                 # FastAPI app
│   ├── requirements.txt
│   ├── src/
│   │   ├── api/
│   │   │   └── chatbot.py     # API endpoints
│   │   ├── services/
│   │   │   ├── chatbot_service.py
│   │   │   ├── qdrant_service.py
│   │   │   ├── embedding_service.py
│   │   │   ├── llm_service.py
│   │   │   ├── db_service.py
│   │   │   └── content_processor.py
│   │   └── models/
│   │       └── schemas.py
│   └── scripts/
│       └── ingest_content.py
├── physical-ai-textbook/
│   └── src/
│       └── components/
│           ├── ChatWidget.jsx
│           └── ChatWidget.module.css
└── .env
```

## API Endpoints

### POST /api/ask
Ask a question

Request:
```json
{
  "question": "What is a ROS 2 node?",
  "selected_text": null,
  "current_page": "/week-03-ros2-part1/03-first-python-node",
  "session_id": "session-123"
}
```

Response:
```json
{
  "answer": "A ROS 2 node is...",
  "citations": [
    {
      "title": "Week 3: ROS 2 Part 1",
      "url": "/week-03-ros2-part1/03-first-python-node",
      "section": "Introduction to Nodes",
      "snippet": "..."
    }
  ],
  "session_id": "session-123"
}
```

### GET /api/health
Check service health

## Deployment

### Backend (Render)

1. Create new Web Service
2. Connect your repo
3. Build: `cd backend && pip install -r requirements.txt`
4. Start: `cd backend && python main.py`
5. Add environment variables

### Frontend (GitHub Pages)

Already deployed with Docusaurus. The chat widget is embedded.

## Troubleshooting

**Qdrant connection fails:**
- For local: Install Qdrant with Docker: `docker run -p 6333:6333 qdrant/qdrant`
- For cloud: Check URL and API key in `.env`

**Database connection fails:**
- Verify `DATABASE_URL` in `.env`
- Check Neon dashboard for connection string

**Content not indexed:**
- Run `python scripts/ingest_content.py` again
- Check `DOCS_DIR` path in `.env`

**Chat widget not showing:**
- Ensure `Root.js` is created
- Check browser console for errors
- Verify backend is running

## License

MIT
