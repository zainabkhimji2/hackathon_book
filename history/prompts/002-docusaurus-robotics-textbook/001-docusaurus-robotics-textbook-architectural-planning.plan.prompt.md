---
id: 001
title: Docusaurus Robotics Textbook Architectural Planning
stage: plan
date: 2025-11-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-docusaurus-robotics-textbook
branch: 002-docusaurus-robotics-textbook
user: system
command: /sp.plan
labels: ["architecture", "planning", "docusaurus", "robotics", "ros2", "education"]
links:
  spec: specs/002-docusaurus-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-docusaurus-robotics-textbook/research.md
  - specs/002-docusaurus-robotics-textbook/data-model.md
  - specs/002-docusaurus-robotics-textbook/contracts/chatbot-api.yaml
  - specs/002-docusaurus-robotics-textbook/contracts/README.md
  - specs/002-docusaurus-robotics-textbook/quickstart.md
  - specs/002-docusaurus-robotics-textbook/plan.md
tests:
  - Constitution compliance check (PASS)
  - Technical context validation (PASS - all NEEDS CLARIFICATION resolved)
  - Data model entity validation (9 entities defined)
  - API contract validation (OpenAPI 3.1 spec)
---

## Prompt

Create a comprehensive architectural plan for the Docusaurus skeleton with 3 chapters.

TECH STACK DECISIONS:

Frontend Framework:
- Docusaurus 3.x (latest stable)
- React 18.x with TypeScript
- Reason: Production-ready docs platform, built-in features (search, versioning, i18n ready), excellent performance, React-based for future chatbot integration

Styling:
- Docusaurus custom CSS + CSS modules
- Tailwind CSS NOT used (stick to Docusaurus theming system)
- Custom color palette: Primary (#0ea5e9 cyan), Secondary (#1e293b slate)

Content Format:
- MDX (Markdown + JSX components)
- Mermaid for diagrams
- Prism for code highlighting with ROS 2 language support

PROJECT STRUCTURE:
physical-ai-textbook/
├── docs/
│   ├── intro.md
│   ├── week-01-02-intro-physical-ai/
│   │   ├── category.json
│   │   ├── index.md
│   │   ├── 01-foundations.md
│   │   ├── 02-embodied-intelligence.md
│   │   ├── 03-sensor-systems.md
│   │   ├── 04-physical-vs-digital-ai.md
│   │   └── exercises.md
│   ├── week-03-ros2-part1/
│   │   ├── category.json
│   │   ├── index.md
│   │   ├── 01-ros2-architecture.md
│   │   ├── 02-nodes-topics.md
│   │   ├── 03-first-python-node.md
│   │   ├── 04-publisher-subscriber.md
│   │   └── exercises.md
│   ├── week-04-ros2-part2/
│   │   ├── category.json
│   │   ├── index.md
│   │   ├── 01-services-actions.md
│   │   ├── 02-simple-pub-sub.md
│   │   ├── 03-message-types.md
│   │   ├── 04-launch-files.md
│   │   └── exercises.md
│   └── resources/
│       ├── glossary.md
│       └── setup-guide.md
├── src/
│   ├── components/
│   │   ├── HomepageFeatures/
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   └── CodeBlock/
│   │       └── index.tsx
│   ├── css/
│   │   └── custom.css
│   └── pages/
│       ├── index.tsx
│       └── index.module.css
├── static/
│   ├── img/
│   │   ├── logo.svg
│   │   ├── favicon.ico
│   │   └── social-card.png
│   └── robots.txt
├── docusaurus.config.ts
├── sidebars.ts
├── package.json
├── tsconfig.json
└── .gitignore

[Full user prompt containing all technical specifications, configuration details, content architecture, deployment architecture, performance optimization, accessibility requirements, content creation workflow, diagram strategy, mobile optimization, RAG readiness, testing strategy, performance optimization, accessibility implementation, companion code repository structure, risk mitigation, definition of done, and follow-up phases]

## Response snapshot

Executed comprehensive `/sp.plan` workflow to create architectural plan for Docusaurus-based Physical AI & Humanoid Robotics interactive textbook.

**Phase 0: Research & Technical Decisions (COMPLETE)**
Generated `research.md` with 17 critical technical decisions:
1. Framework: Docusaurus 3.6.3 (over Next.js/VuePress) - production-ready docs platform with built-in features
2. Search: Algolia DocSearch free tier with client-side fallback
3. Code Highlighting: Prism.js with custom ROS 2 language support
4. Diagrams: Mermaid.js for architecture, text placeholders for custom illustrations
5. Deployment: GitHub Actions → GitHub Pages (zero-cost hosting, automated CI/CD)
6. Theme: Custom CSS with robotics color scheme (cyan #0ea5e9, slate #1e293b)
7. Content Structure: Nested hierarchy (Chapter → Section → Subsection) with separate markdown files
8. Mobile: Mobile-first responsive design targeting 360px+ width
9. Performance: Aggressive code splitting, lazy loading, WebP images, bundle size <150KB
10. Accessibility: WCAG 2.1 Level AA compliance with Lighthouse CI enforcement (95+ scores)
11. Content Workflow: Agent-driven (educational-content-designer → lesson-writer → ros2-expert → technical-reviewer)
12. Testing: Multi-layer validation (code in ROS 2 Docker, link checking, accessibility audits, performance monitoring)
13. Companion Repository: Separate GitHub repo for complete runnable ROS 2 code examples
14. RAG Preparation: Clean markdown with structured frontmatter metadata for semantic chunking

All NEEDS CLARIFICATION items resolved with concrete technical specifications.

**Phase 1: Design & Contracts (COMPLETE)**
Generated complete design artifacts:

1. **data-model.md**: Defined 9 content entities with validation rules
   - Chapter (learning module for 1-2 weeks)
   - Section (focused topic page with frontmatter)
   - Code Example (runnable ROS 2 snippets with repository links)
   - Exercise (hands-on practice with hints and solutions)
   - Diagram (Mermaid.js visualizations)
   - Cross-Reference (inter-section links)
   - Glossary Term (domain-specific definitions)
   - Navigation Item (sidebar structure)
   - Theme Preference (light/dark mode state)
   - Entity relationship diagram with ERD in Mermaid format
   - Content lifecycle and RAG ingestion format

2. **contracts/chatbot-api.yaml**: OpenAPI 3.1 specification for Phase 2 RAG chatbot
   - POST /chat - Question answering with citations
   - POST /chat/stream - Server-sent events streaming
   - GET /chat/sessions/{id} - Conversation history
   - GET /content/search - Semantic search across textbook
   - GET /content/sections/{id} - Section retrieval
   - GET /content/related - Related section discovery
   - Complete request/response schemas with examples
   - Error handling patterns and rate limiting

3. **contracts/README.md**: API integration documentation
   - Usage examples for React frontend and FastAPI backend
   - Client code generation instructions
   - Mock server setup for development
   - Data flow sequence diagrams
   - Versioning strategy

4. **quickstart.md**: Contributor setup guide
   - Prerequisites and installation steps
   - Project structure explanation
   - Development workflow (content writing, code examples, diagrams)
   - Available NPM scripts
   - Testing procedures (local build, ROS 2 code validation, accessibility checks)
   - Common issues and solutions
   - Configuration file explanations
   - Content guidelines and best practices

**Implementation Plan (plan.md) - COMPLETE**
Filled complete architectural plan with:
- Summary: Production-ready Docusaurus textbook with 3 chapters, ROS 2 examples, RAG-optimized content
- Technical Context: TypeScript 5.7.2, Docusaurus 3.6.3, React 18.2.0, Mermaid.js, static SSG architecture
- Performance Goals: FCP <1.5s, TTI <3s, Lighthouse 95+
- Constraints: GitHub Pages free tier, static hosting only, 10min build timeout
- Scale: Initial 3 chapters (~30 pages), future 13 weeks (~130 pages), 1K-5K students/year

**Constitution Check: PASS ✅**
All 6 core principles satisfied:
- I. Educational Excellence: Agent-driven content workflow, WCAG 2.1 AA compliance
- II. Technical Architecture: React functional components, TypeScript, hooks, accessibility
- III. Security: No user data in Phase 1, static site inherently secure
- IV. Performance: Targets met with code splitting, lazy loading, CDN
- V. DevOps: GitHub Actions CI/CD, automated validation, deployment to Pages
- VI. Project Management: Conventional commits, atomic changes, task tracking

**Project Structure**: Web documentation site (static SSG)
- Primary repo: Docusaurus textbook (markdown content in `docs/`, React components in `src/`)
- Companion repo: Runnable ROS 2 code examples (tested in Docker CI/CD)
- Deployment: `build/` directory → GitHub Pages

**Risk Mitigation**:
1. Content quality → Agent validation chain
2. ROS 2 code failures → Mandatory Docker testing
3. Free tier limits → Monitoring + Cloudflare fallback
4. Search issues → Algolia + client-side fallback
5. Mobile performance → Lighthouse CI blocking merge

**ADR Candidates** (3 decisions pending formal documentation):
1. Docusaurus over Next.js - Framework choice
2. TypeScript over JavaScript - Type safety
3. Incremental content (3 chapters first) - Delivery strategy

**Next Steps**:
1. User reviews and approves plan
2. Document ADRs: `/sp.adr [decision-title]` for each
3. Generate tasks: `/sp.tasks` for dependency-ordered implementation
4. Execute: Manual or `/sp.implement` for autonomous task processing

## Outcome

- ✅ Impact: Complete architectural plan for Docusaurus-based Physical AI textbook with all technical unknowns resolved and design artifacts generated
- 🧪 Tests: Constitution compliance (PASS), Technical validation (all NEEDS CLARIFICATION resolved), Data model validation (9 entities), API contract validation (OpenAPI 3.1)
- 📁 Files: research.md, data-model.md, contracts/chatbot-api.yaml, contracts/README.md, quickstart.md, plan.md (6 artifacts)
- 🔁 Next prompts:
  - `/sp.adr Docusaurus-over-Next.js` - Document framework choice
  - `/sp.adr TypeScript-over-JavaScript` - Document type safety decision
  - `/sp.adr Incremental-content-approach` - Document delivery strategy
  - `/sp.tasks` - Generate implementation tasks from plan
- 🧠 Reflection: Planning workflow executed successfully with all phases complete. Research phase identified and resolved 17 critical technical decisions. Design phase produced comprehensive data model (9 entities) and API contracts for future RAG integration. Constitution check passed all 6 core principles with no violations. Ready for task generation and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning phases completed successfully
- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (all 6 principles satisfied)
  - Technical Context: PASS (all placeholders filled with concrete specs)
  - Research Completeness: PASS (17 decisions documented, zero NEEDS CLARIFICATION remaining)
  - Data Model Validation: PASS (9 entities with relationships, validation rules, RAG format)
  - API Contract Validation: PASS (OpenAPI 3.1 spec valid, complete schemas)
- Prompt variant (if applicable): Standard /sp.plan workflow from .specify/templates/commands/plan.md
- Next experiment (smallest change to try): None needed - plan complete and validated. Proceed to task generation with `/sp.tasks`.
