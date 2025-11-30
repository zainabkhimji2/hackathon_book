
# Physical AI & Humanoid Robotics Interactive Textbook

## 🎯 PROJECT OVERVIEW

**What This Project Does:**
An AI-native educational platform that teaches Physical AI, embodied intelligence, ROS 2, humanoid robotics, and NVIDIA Isaac through an interactive Docusaurus textbook with embedded RAG chatbot, user authentication, content personalization, and Urdu translation.

**Problem It Solves:**
Students learning robotics face fragmented resources, lack of hands-on guidance, and difficulty connecting software concepts to physical robots. This textbook provides a comprehensive, personalized learning path from ROS 2 basics to advanced humanoid control with AI assistance at every step.

**Target Audience:**
- Beginners with basic programming knowledge
- Software engineers transitioning to robotics
- Hardware engineers learning ROS 2
- Students in robotics courses
- Self-learners exploring physical AI

**Key Features:**
- 13 weeks of comprehensive robotics curriculum (ROS 2, Gazebo, Isaac Sim, humanoid control)
- RAG chatbot with text-selection Q&A capability
- User authentication with background profiling
- Content personalization based on user experience level
- Urdu translation for accessibility
- 100+ runnable code examples
- Hands-on exercises and projects

---

## 🛠️ TECHNOLOGY STACK

**Frontend:**
- Docusaurus 3.x (documentation platform)
- React 18 with TypeScript (UI components)
- React Hooks (useState, useEffect, useContext, custom hooks)
- CSS3 + Docusaurus theming (styling)
- MDX (interactive documentation)

**Backend:**
- Python 3.11+ (FastAPI framework)
- FastAPI (REST API, async endpoints)
- Pydantic (request/response validation)
- OpenAI Python SDK (GPT-4, embeddings)
- OpenAI Agents SDK (agentic chatbot)
- rclpy (ROS 2 Python client library)

**AI & ML:**
- OpenAI GPT-4 Turbo (chatbot responses, personalization, translation)
- OpenAI text-embedding-3-small (vector embeddings for RAG)
- ChatKit SDK (chat UI components)

**Databases:**
- Neon Serverless Postgres (user data, conversations, progress tracking)
- Qdrant Cloud Free Tier (vector database for RAG embeddings)

**Authentication:**
- Better-auth or custom JWT implementation
- bcrypt (password hashing)
- HTTP-only cookies (token storage)

**DevOps & Deployment:**
- GitHub Actions (CI/CD pipeline)
- GitHub Pages or Vercel (frontend hosting)
- Render or Railway (backend hosting)
- Git (version control)

**Development Tools:**
- Claude Code CLI (AI-assisted development)
- Spec-Kit Plus (specification-driven development)
- Node.js 18+ and npm (package management)
- Python venv (virtual environment)

**Key Dependencies:**
```json
// Frontend (package.json)
{
  "@docusaurus/core": "^3.x",
  "@docusaurus/preset-classic": "^3.x",
  "@docusaurus/theme-mermaid": "^3.x",
  "react": "^18.x",
  "react-dom": "^18.x",
  "typescript": "^5.x"
}

// Backend (requirements.txt)
fastapi==0.104.1
uvicorn[standard]==0.24.0
openai==1.3.0
qdrant-client==1.7.0
psycopg2-binary==2.9.9
python-dotenv==1.0.0
pydantic==2.5.0
bcrypt==4.1.1
python-jose[cryptography]==3.3.0
```

---

## 📁 DIRECTORY STRUCTURE
```
physical-ai-textbook/
├── docs/                          # Docusaurus documentation source
│   ├── intro.md                   # Course landing page
│   ├── week-01-02-intro-physical-ai/
│   │   ├── index.md
│   │   ├── foundations.md
│   │   ├── embodied-intelligence.md
│   │   └── sensor-systems.md
│   ├── week-03-05-ros2-fundamentals/
│   │   ├── index.md
│   │   ├── architecture.md
│   │   ├── nodes-topics.md
│   │   └── python-packages.md
│   ├── week-06-07-gazebo-simulation/
│   ├── week-08-10-nvidia-isaac/
│   ├── week-11-12-humanoid-development/
│   ├── week-13-conversational-robotics/
│   └── resources/
│       ├── hardware-requirements.md
│       ├── glossary.md
│       └── references.md
├── src/
│   ├── components/              # React components
│   │   ├── ChatbotWidget/       # RAG chatbot UI
│   │   │   ├── ChatButton.tsx
│   │   │   ├── ChatPanel.tsx
│   │   │   ├── MessageList.tsx
│   │   │   ├── MessageInput.tsx
│   │   │   └── TextSelectionHandler.tsx
│   │   ├── Auth/                # Authentication components
│   │   │   ├── SignupForm.tsx
│   │   │   ├── SigninForm.tsx
│   │   │   ├── ProfileSetup.tsx
│   │   │   └── AuthContext.tsx
│   │   └── Personalization/
│   │       ├── PersonalizeButton.tsx
│   │       └── TranslateButton.tsx
│   ├── css/
│   │   └── custom.css
│   └── pages/
│       └── index.tsx            # Homepage
├── backend/                     # FastAPI backend
│   ├── main.py                  # FastAPI app entry point
│   ├── api/
│   │   ├── chat.py              # Chatbot endpoints
│   │   ├── auth.py              # Authentication endpoints
│   │   ├── personalize.py       # Personalization endpoints
│   │   └── translate.py         # Translation endpoints
│   ├── services/
│   │   ├── rag_service.py       # RAG pipeline logic
│   │   ├── embedding_service.py # OpenAI embeddings
│   │   ├── qdrant_manager.py    # Vector DB operations
│   │   └── db_manager.py        # Postgres operations
│   ├── models/
│   │   ├── schemas.py           # Pydantic models
│   │   └── database.py          # SQLAlchemy models
│   ├── ingestion/
│   │   ├── content_parser.py    # Parse markdown files
│   │   ├── chunk_processor.py   # Chunk content
│   │   └── ingest.py            # Ingestion pipeline script
│   ├── middleware/
│   │   ├── auth.py              # JWT authentication middleware
│   │   └── cors.py              # CORS configuration
│   └── config.py                # Environment configuration
├── static/
│   ├── img/                     # Images and icons
│   └── videos/                  # Demo videos
├── .github/
│   └── workflows/
│       └── deploy.yml           # CI/CD pipeline
├── docusaurus.config.ts         # Docusaurus configuration
├── sidebars.ts                  # Sidebar navigation config
├── package.json                 # Frontend dependencies
├── requirements.txt             # Backend dependencies
├── .env.example                 # Environment variables template
├── .gitignore
├── README.md
└── CLAUDE.md                    # This file (persistent context)
```

---

## ✨ CODING CONVENTIONS

**General Principles:**
- Write clean, readable, self-documenting code
- Follow DRY (Don't Repeat Yourself) principle
- Use meaningful variable and function names
- Comment complex logic, not obvious code
- Prioritize type safety and error handling
- Write modular, reusable code

**Python (Backend):**
- Follow PEP 8 style guide
- Use type hints for all function parameters and return types
- Use async/await for I/O operations
- Prefer f-strings for string formatting
- Use context managers (with statements) for resources
- Handle exceptions explicitly with try-except
- Use Pydantic for data validation
- Keep functions under 50 lines
- Use descriptive names: `get_user_by_email()` not `get_user()`
- Constants in UPPER_CASE: `MAX_TOKEN_LENGTH = 500`

**Example:**
```python
async def get_conversation_history(
    conversation_id: str,
    limit: int = 50
) -> List[Message]:
    """Retrieve conversation history from database.
    
    Args:
        conversation_id: Unique conversation identifier
        limit: Maximum number of messages to return
        
    Returns:
        List of Message objects ordered by timestamp
    """
    try:
        # Implementation
        pass
    except DatabaseError as e:
        logger.error(f"Failed to fetch conversation: {e}")
        raise HTTPException(status_code=500, detail="Database error")
```

**TypeScript/React (Frontend):**
- Use functional components with hooks (no class components)
- Define interfaces for all component props
- Use meaningful component names: `ChatMessage` not `Msg`
- Extract reusable logic into custom hooks
- Keep components under 200 lines
- Use const for all variables unless reassignment needed
- Prefer arrow functions: `const handleClick = () => {}`
- Use optional chaining: `user?.profile?.name`
- Always include proper accessibility attributes (aria-label, role)

**Example:**
```typescript
interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
  conversationId?: string;
}

export const ChatPanel: React.FC<ChatPanelProps> = ({ 
  isOpen, 
  onClose, 
  conversationId 
}) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  
  // Implementation
  
  return (
    <div className="chat-panel" aria-label="Chat interface">
      {/* JSX */}
    </div>
  );
};
```

**ROS 2 (Python rclpy):**
- Follow ROS 2 naming conventions
- Node names: lowercase with underscores: `velocity_controller`
- Topic names: lowercase with forward slashes: `/robot/cmd_vel`
- Use QoS profiles appropriately (Reliable vs Best Effort)
- Implement proper lifecycle management
- Always include shutdown handlers
- Log at appropriate levels (DEBUG, INFO, WARN, ERROR)

**Example:**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Velocity publisher initialized')
```

**Error Handling:**
- Always handle errors, never let them propagate silently
- Return meaningful error messages to users
- Log errors with context
- Use proper HTTP status codes (200, 201, 400, 401, 404, 500)
- Implement retry logic for transient failures

**Security:**
- Never log sensitive data (passwords, tokens, API keys)
- Use parameterized queries for all database operations
- Validate all user inputs
- Sanitize data before rendering
- Use HTTPS in production
- Store secrets in environment variables, never in code

**Git Commits:**
- Use conventional commit format: `feat:`, `fix:`, `docs:`, `refactor:`
- Keep commits atomic (one logical change per commit)
- Write descriptive commit messages
- Example: `feat: add text selection Q&A to chatbot widget`

---

## 🚀 KEY COMMANDS

**Development:**
```bash
# Frontend (Docusaurus)
npm install                    # Install dependencies
npm start                      # Start dev server (localhost:3000)
npm run build                  # Production build
npm run serve                  # Serve production build locally
npm run clear                  # Clear cache

# Backend (FastAPI)
python -m venv venv            # Create virtual environment
source venv/bin/activate       # Activate venv (Linux/Mac)
venv\Scripts\activate          # Activate venv (Windows)
pip install -r requirements.txt # Install dependencies
uvicorn main:app --reload     # Start dev server (localhost:8000)
python backend/ingestion/ingest.py  # Run content ingestion

# Database
psql $DATABASE_URL             # Connect to Neon Postgres
python -m alembic upgrade head # Run migrations
```

**Testing:**
```bash
# Frontend
npm run test                   # Run tests

# Backend
pytest                         # Run all tests
pytest tests/test_auth.py      # Run specific test file
pytest -v                      # Verbose output
```

**Deployment:**
```bash
# Frontend (automatic via GitHub Actions on push to main)
git push origin main           # Triggers deployment

# Backend (manual deploy to Render/Railway)
git push render main           # Deploy to Render
railway up                     # Deploy to Railway

# Environment setup
cp .env.example .env           # Create env file
# Then edit .env with actual values
```

**Claude Code Specific:**
```bash
# Subagent usage
/subagent educational-content-designer  # Use content designer
/subagent lesson-writer                 # Use lesson writer
/subagent python-expert                 # Use Python expert
/subagent react-developer               # Use React developer

# Skill activation
/skill code-example-generator   # Generate code examples
/skill rag-pipeline            # RAG implementation patterns
/skill authentication-flow     # Auth patterns
```

---

## ⚠️ IMPORTANT NOTES

**Critical Constraints:**

1. **Timeline:** Hackathon submission deadline is Nov 30, 2025, 6:00 PM. Prioritize base features (100 points) before bonuses.

2. **API Costs:** OpenAI API costs can spike. Implement aggressive caching, use GPT-3.5-turbo for development, minimize embedding calls.

3. **Free Tier Limits:**
   - Neon Postgres: 10GB storage, 100 hours compute/month
   - Qdrant Cloud: 1GB vector storage
   - Render/Railway: Limited compute on free tier
   - Plan chunking and caching strategies carefully

4. **Security Requirements:**
   - Never commit secrets to GitHub
   - Use `.env` for all secrets (OPENAI_API_KEY, DATABASE_URL, etc.)
   - Add `.env` to `.gitignore`
   - Include `.env.example` with dummy values
   - Hash passwords with bcrypt (12+ rounds minimum)
   - Use HTTP-only cookies for JWT tokens

5. **Content Accuracy:**
   - All ROS 2 code must work with Humble/Iron
   - Verify hardware specifications are current and purchasable
   - Test all code examples before publishing
   - Use technical-reviewer subagent for validation

6. **Performance Targets:**
   - Page load: <2s on 4G
   - Chatbot response: <3s average
   - Database queries: <100ms
   - Vector search: <500ms
   - Mobile responsive on 360px+ screens

7. **RAG Pipeline Gotchas:**
   - Chunk size: 300-700 tokens (sweet spot for balance)
   - Overlap: 50-100 tokens to preserve context
   - Preserve code blocks intact (don't split mid-code)
   - Include metadata: page title, section, week
   - Top-k: Start with 5, tune based on relevance
   - Always cite sources in responses

8. **ROS 2 Development:**
   - Always test in simulation before real hardware
   - Implement safety checks (velocity limits, collision avoidance)
   - Use proper QoS profiles (Reliable for critical topics)
   - Include shutdown handlers to clean up resources
   - Validate URDF before loading in Gazebo

9. **Deployment Checklist:**
   - Test production build locally first
   - Verify all environment variables set
   - Check CORS settings for production domain
   - Test on mobile devices
   - Run accessibility audit (Lighthouse)
   - Create database backups
   - Test rollback procedure

10. **Common Pitfalls to Avoid:**
    - Forgetting to activate virtual environment (Python)
    - Not clearing Docusaurus cache after config changes
    - Hardcoding localhost URLs (use env vars)
    - Skipping error handling (always handle edge cases)
    - Not testing text selection on different browsers
    - Forgetting to update sidebars.ts after adding pages
    - Not implementing rate limiting (API abuse risk)

**Environment Variables:**
```bash
# Backend (.env file)
OPENAI_API_KEY=sk-...
DATABASE_URL=postgresql://user:pass@host/db
QDRANT_URL=https://...
QDRANT_API_KEY=...
JWT_SECRET=random_32_char_string
FRONTEND_URL=http://localhost:3000

# Frontend (.env file)
REACT_APP_API_URL=http://localhost:8000
```

**Troubleshooting:**
- If build fails: Clear cache (`npm run clear`, delete `node_modules`, reinstall)
- If API calls fail: Check CORS settings, verify env vars loaded
- If embedding search returns poor results: Adjust chunk size, try different top-k
- If database connection fails: Verify DATABASE_URL format, check Neon dashboard
- If chatbot is slow: Check OpenAI API status, implement caching

**Useful Resources:**
- Docusaurus Docs: https://docusaurus.io/docs
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- FastAPI Docs: https://fastapi.tiangolo.com/
- OpenAI API Reference: https://platform.openai.com/docs/api-reference
- Qdrant Documentation: https://qdrant.tech/documentation/
- Neon Documentation: https://neon.tech/docs

---

**This CLAUDE.md file is loaded automatically at every Claude Code session start. Update it as the project evolves. Last updated: [Current Date]**