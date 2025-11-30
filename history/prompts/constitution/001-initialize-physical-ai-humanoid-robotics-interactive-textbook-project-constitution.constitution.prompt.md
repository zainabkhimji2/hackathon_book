---
id: 001
title: Initialize Physical AI & Humanoid Robotics Interactive Textbook Project Constitution
stage: constitution
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: User
command: /sp.constitution
labels: ["project-setup", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
null
---

## Prompt

```
 As an expert architect, generate a detailed project constitution for the "Physical AI & Humanoid Robotics Interactive Textbook"
   project. This constitution should serve as the foundational principles document, guiding all development, content creation,
  and operational decisions. **It should be structured not only around the principles below but also align with the "Architect
  Guidelines" provided in the `CLAUDE.md` file, where applicable, to ensure a robust and comprehensive architectural
  foundation.**

  Thoroughly address the following sections, integrating relevant project details, technology stack, coding conventions, and
  critical constraints from `PROJECT_INFO.md`, and considering the capabilities of the available subagents and skills:

  1.  **Core Project Values & Mission:**
      *   Clearly articulate the project's purpose, target audience, and the unique problem it solves (fragmented resources, lack
   of hands-on guidance).
      *   Emphasize interactivity, personalization, accessibility (Urdu translation), and comprehensive learning.

  2.  **Educational Content Principles:**
      *   Define guidelines for content quality, accuracy, and pedagogical effectiveness.
      *   Specify how the 13-week curriculum, code examples, exercises, and projects should be designed and implemented to ensure
   clarity, engagement, and hands-on learning.
      *   Detail the role of the `educational-content-designer`, `lesson-writer`, `technical-reviewer`, `ros2-expert`, and
       `robotics-curriculum-designer` subagents in maintaining content integrity and educational value.
      *   Incorporate principles related to `learning-objectives`, `concept-scaffolding`, `code-example-generator`,
       `exercise-designer`, `assessment-builder`, `technical-clarity`, `book-scaffolding`, and `ai-augmented-teaching` skills.

  3.  **Technical Architecture & Implementation Standards:**
      *   **Frontend (Docusaurus/React/TypeScript):** Principles for UI/UX design, component development (functional, hooks),
       state management, accessibility, and theming. Reference the `react-developer` subagent and `react-components` skill.
      *   **Backend (FastAPI/Python):** Standards for REST API design, async programming, data validation (Pydantic), and
       modularity. Reference the `python-expert` and `api-architect` subagents, and `fastapi-development` skill.
      *   **AI/ML & RAG Pipeline:** Guidelines for integrating OpenAI models, managing embeddings (Qdrant), chatbot responses,
       personalization, and translation. Emphasize RAG pipeline best practices (chunking, overlap, metadata, citation) and cost
       optimization. Reference the `rag-pipeline` skill.
      *   **Databases (Postgres/Qdrant):** Principles for schema design, data integrity, query optimization, and data management
       in both relational (Neon) and vector (Qdrant) databases. Reference the `database-specialist` subagent and `database-design`
       skill.
      *   **Authentication & Authorization:** Standards for user authentication (JWT, bcrypt, HTTP-only cookies), user profiling,
        and secure token management. Reference the `authentication-flow` skill.
      *   **ROS 2 & Robotics Development:** Principles for ROS 2 naming conventions, QoS profiles, lifecycle management, safety
       checks, simulation-first approach, and URDF validation. Reference the `ros2-expert` subagent.

  4.  **Security & Data Privacy Principles:**
      *   Establish strict guidelines for handling sensitive data, API keys, and user information.
      *   Detail requirements for environment variable usage, password hashing, input validation, data sanitization, and secure
       deployment.

  5.  **Performance & Scalability Targets:**
      *   Define measurable targets for page load times, chatbot response latency, database queries, and vector search.
      *   Outline strategies for achieving these targets, including caching, efficient queries, and mobile responsiveness.

  6.  **DevOps & Deployment Strategy:**
      *   Principles for CI/CD using GitHub Actions, frontend/backend hosting, and ensuring operational readiness (observability,
        alerting, runbooks, deployment/rollback). Reference the `github-actions` skill.

  7.  **Project Management & Collaboration:**
      *   Define conventions for Git commits (conventional commits, atomic changes).
      *   Emphasize working within hackathon constraints (timeline, free tier limits).
      *   Outline processes for task tracking and using available agents and skills effectively.

  8.  **Risk Management & Mitigation:**
      *   Identify top potential risks (e.g., API costs, free tier limits, content accuracy, security vulnerabilities) and
       outline mitigation strategies.

  9.  **Architectural Decision Records (ADR) Policy:**
      *   Define criteria for architecturally significant decisions and the process for documenting them using `/sp.adr`.

  Ensure the constitution is concise, actionable, and reflects a balance between technical excellence, educational effectiveness,
        and project constraints also maintain all the history of what i prompt you in a history as well to record all the prompt and history
```

## Response snapshot

```
Generated the initial project constitution for "Physical AI & Humanoid Robotics Interactive Textbook" and saved it to `.specify/memory/constitution.md`.
New Version: 1.0.0 (Initial creation)
Files flagged for manual follow-up for alignment:
- .specify/templates/plan-template.md
- .specify/templates/spec-template.md
- .specify/templates/tasks-template.md
Suggested commit message: `docs: feat: initialize project constitution to v1.0.0`
```

## Outcome

- ✅ Impact: Initial project constitution created and saved.
- 🧪 Tests: No tests run as this was a documentation generation task.
- 📁 Files: One file created/modified: .specify/memory/constitution.md
- 🔁 Next prompts: User to review constitution and update flagged templates if necessary.
- 🧠 Reflection: Successfully generated constitution based on detailed prompt and project info. Ensured alignment with architectural guidelines. Handled script not found gracefully by falling back to agent-native PHR creation. Flagged dependent templates for manual review.

## Evaluation notes (flywheel)

- Failure modes observed: create-phr.sh script not found, successfully handled by agent-native PHR creation.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment: Improve script discovery or provide a default script location.
