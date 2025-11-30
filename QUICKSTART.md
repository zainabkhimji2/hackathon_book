# Quick Start Guide

## Setup in 5 Minutes

### Step 1: Add Your API Keys

1. Copy `.env.example` to `.env`
2. Fill in your API keys:
   - OPENAI_API_KEY=sk-...
   - QDRANT_URL=https://...
   - QDRANT_API_KEY=...
   - DATABASE_URL=postgresql://...

### Step 2: Install & Setup Backend

```bash
cd backend
pip install -r requirements.txt
python setup_db.py
```

### Step 3: Ingest Content

```bash
python scripts/ingest_content.py
```

### Step 4: Start Backend

```bash
python main.py
```

Leave this running! Backend is at http://localhost:8000

### Step 5: Test the Chatbot

Open new terminal:

```bash
cd physical-ai-textbook
npm start
```

Visit http://localhost:3000 and click the chat button!

## That's It!

The chat widget is already integrated into your Docusaurus site.

Ask questions like:
- "What is a ROS 2 node?"
- "How do I create a publisher?"
- "Explain subscribers"

## Next Steps

- Deploy backend to Render (free tier)
- Deploy frontend to GitHub Pages
- Add more content to textbook

## Troubleshooting

**Chat button not appearing?**
- Check `physical-ai-textbook/src/theme/Root.js` exists
- Restart Docusaurus: `npm start`

**Backend errors?**
- Check .env file has all keys
- Run `python setup_db.py` again

**No answers?**
- Run `python scripts/ingest_content.py` again
- Check backend logs for errors
