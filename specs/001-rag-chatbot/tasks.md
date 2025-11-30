# Feature Tasks: RAG-Powered AI Chatbot for Physical AI Textbook

**Feature Branch**: `001-rag-chatbot` | **Date**: 2025-11-30 | **Plan**: /specs/001-rag-chatbot/plan.md

## Implementation Strategy

This implementation will follow an MVP-first approach, prioritizing core chatbot functionality (asking questions and getting cited answers, and accessible widget) before enhancing with text selection and multi-turn conversations. Tasks are ordered by user story priority and within each story, by logical dependency.

## Dependencies

- User Story 1 (Ask Questions and Get Cited Answers) is foundational.
- User Story 4 (Accessible Chatbot Widget) is largely independent but enhances the core experience.
- User Story 2 (Text Selection Question Mode) depends on User Story 1's core functionality.
- User Story 3 (Multi-Turn Contextual Conversations) depends on User Story 1.

## Phase 1: Setup (Project Initialization)

- [ ] T001 Create backend directory `backend/` and initialize FastAPI project
- [ ] T002 Create frontend directory `frontend/` and integrate into Docusaurus structure
- [ ] T003 Configure environment variables for API keys and database connections

## Phase 2: Foundational (Blocking Prerequisites)

- [ ] T004 Implement Qdrant client connection and basic health check in `backend/src/services/qdrant_service.py`
- [ ] T005 Implement Neon Serverless Postgres client connection for conversation history in `backend/src/services/db_service.py`
- [ ] T006 Develop content ingestion script for markdown files in `docs/` (`backend/scripts/ingest_content.py`)
- [ ] T007 Implement chunking logic (h2/h3 headers, token overlap, code blocks) in `backend/src/services/content_processor.py`
- [ ] T008 Implement OpenAI embedding generation for content chunks in `backend/src/services/embedding_service.py`
- [ ] T009 Store content embeddings and metadata in Qdrant (initial indexing) using `backend/scripts/ingest_content.py`

## Phase 3: User Story 1 - Ask Questions and Get Cited Answers [P1]
*Independent Test: Ask "What is a ROS 2 node?" and verify an accurate answer with clickable citations to textbook sections.*

- [ ] T010 [US1] Create FastAPI endpoint `/ask` for question answering in `backend/src/api/chatbot.py`
- [ ] T011 [P] [US1] Integrate OpenAI LLM for response generation in `backend/src/services/llm_service.py`
- [ ] T012 [P] [US1] Implement vector search in Qdrant to retrieve relevant chunks based on query in `backend/src/services/qdrant_service.py`
- [ ] T013 [US1] Develop logic to construct RAG prompt with retrieved chunks and conversation context in `backend/src/services/chatbot_service.py`
- [ ] T014 [US1] Extract and format citations (page title, URL, section) from retrieved chunks in `backend/src/services/chatbot_service.py`
- [ ] T015 [US1] Create a basic Chatbot UI component (floating button, expandable panel, input field, message display) in `frontend/src/components/ChatWidget.tsx`
- [ ] T016 [US1] Implement API call from frontend to backend `/ask` endpoint in `frontend/src/services/chatbot_api.ts`
- [ ] T017 [US1] Display chatbot responses with markdown rendering and clickable citation links in `frontend/src/components/MessageDisplay.tsx`
- [ ] T018 [US1] Integrate `ChatWidget` into Docusaurus site (`frontend/src/pages/index.tsx` or similar)

## Phase 4: User Story 4 - Accessible Chatbot Widget [P1]
*Independent Test: Load any textbook page on desktop and mobile; verify button appears, panel expands smoothly, and doesn't obstruct content.*

- [ ] T019 [P] [US4] Implement responsive design for chat widget (desktop: 400x600px, mobile: fullscreen) in `frontend/src/components/ChatWidget.tsx` (CSS modules)
- [ ] T020 [P] [US4] Add smooth slide-in/out animations for chat panel in `frontend/src/components/ChatWidget.tsx` (CSS modules)
- [ ] T021 [US4] Implement dark/light mode adaptation matching Docusaurus theme in `frontend/src/components/ChatWidget.tsx` (React Context/CSS vars)
- [ ] T022 [US4] Ensure non-intrusive placement (bottom-right floating button) and z-index handling in `frontend/src/components/ChatWidget.tsx`

## Phase 5: User Story 2 - Text Selection Question Mode [P2]
*Independent Test: Highlight text, click "Ask about this", type "Explain this", and verify targeted answer with current page priority.*

- [ ] T023 [P] [US2] Implement mouseup/touchend event listener to detect text selection in `frontend/src/services/text_selection.ts`
- [ ] T024 [P] [US2] Display "Ask about this" tooltip above selected text in `frontend/src/components/TextSelectionTooltip.tsx`
- [ ] T025 [US2] Modify frontend API call to pass highlighted text as context to backend in `frontend/src/services/chatbot_api.ts`
- [ ] T026 [US2] Enhance backend search logic to prioritize chunks from the current page/section when text context is provided in `backend/src/services/qdrant_service.py`

## Phase 6: User Story 3 - Multi-Turn Contextual Conversations [P3]
*Independent Test: Ask "What is a publisher?" then "How do I create one?" and verify context is maintained.*

- [ ] T027 [US3] Store conversation messages in Neon Postgres (session-scoped) in `backend/src/services/db_service.py`
- [ ] T028 [US3] Retrieve conversation history from DB to pass as context to LLM in `backend/src/services/chatbot_service.py`
- [ ] T029 [US3] Implement React Context or similar for managing and persisting conversation history across components and page navigations in `frontend/src/context/ChatContext.tsx`
- [ ] T030 [US3] Update ChatWidget to display full conversation history in `frontend/src/components/ChatWidget.tsx`

## Phase 7: Polish & Cross-Cutting Concerns

- [ ] T031 Implement robust error handling and loading states for API calls (frontend and backend)
- [ ] T032 Address performance targets (e.g., caching, optimized queries where applicable)
- [ ] T033 Review and implement security measures (CORS, input validation, rate limiting on backend)
- [ ] T034 Add comprehensive unit and integration tests for backend services and API endpoints (`backend/tests/`)
- [ ] T035 Add unit and integration tests for frontend components and services (`frontend/tests/`)
- [ ] T036 Set up CI/CD pipeline (GitHub Actions) for automated testing and deployment to Render/GitHub Pages (`.github/workflows/main.yml`)