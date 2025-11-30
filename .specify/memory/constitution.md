# Physical AI & Humanoid Robotics Interactive Textbook Constitution

## Core Principles

### I. Educational Excellence & Accessibility
Interactive, personalized, comprehensive learning experience (13-week curriculum, runnable examples, exercises). Content must be accurate, pedagogically sound, and accessible (Urdu translation). Leverages `educational-content-designer`, `lesson-writer`, `technical-reviewer`, `ros2-expert`, `robotics-curriculum-designer` subagents and `learning-objectives`, `concept-scaffolding`, `code-example-generator`, `exercise-designer`, `assessment-builder`, `technical-clarity`, `book-scaffolding`, `ai-augmented-teaching` skills.

### II. Robust Technical Architecture
Adherence to defined frontend (Docusaurus/React/TypeScript functional components, hooks, accessibility), backend (FastAPI/Python async, Pydantic, modularity), AI/ML (OpenAI, Qdrant, RAG best practices, cost optimization), and database (Postgres/Qdrant schema, data integrity, query optimization) standards. Leverages `react-developer`, `python-expert`, `api-architect`, `database-specialist` subagents and `react-components`, `fastapi-development`, `rag-pipeline`, `database-design`, `api-integration`, `authentication-flow` skills.

### III. Secure & Private Data Handling
Strict guidelines for sensitive data, API keys, and user information. Mandates environment variables for secrets, bcrypt for password hashing, HTTP-only cookies for JWT, input validation, data sanitization, and secure deployment practices.

### IV. Performance & Scalability Driven Development
Measurable targets for page load (<2s), chatbot response (<3s), DB queries (<100ms), vector search (<500ms), and mobile responsiveness. Strategies include aggressive caching, efficient queries, and optimization for free tier limits.

### V. Operational Readiness & DevOps
Implementation of CI/CD via GitHub Actions. Ensures robust operational readiness through observability (logs, metrics, traces), alerting, runbooks, and defined deployment/rollback strategies. Leverages `github-actions` skill.

### VI. Collaborative & Agile Project Management
Adherence to conventional Git commits (conventional commits, atomic changes, descriptive messages). Emphasizes hackathon constraints (timeline, free tier limits) and structured task tracking using available agents and skills.

## Risk Management & Mitigation

Proactive identification and mitigation of top risks, including OpenAI API costs, free tier limits (Neon, Qdrant, Render/Railway), content accuracy, and security vulnerabilities. Defines kill switches/guardrails where appropriate.

## Architectural Decision Records (ADR) Policy

Architecturally significant decisions (long-term impact, multiple alternatives, cross-cutting scope) must be documented using `/sp.adr [decision-title]` after user consent. Encourages grouping related decisions.

## Governance

Constitution supersedes other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified. Use `CLAUDE.md` and `PROJECT_INFO.md` for runtime development guidance.

**Version**: 1.0.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
