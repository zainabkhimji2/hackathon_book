# Implementation Plan: Docusaurus Robotics Textbook

**Branch**: `002-docusaurus-robotics-textbook` | **Date**: 2025-11-30 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-docusaurus-robotics-textbook/spec.md`

## Summary

Build a production-ready Docusaurus-based interactive textbook for Physical AI & Humanoid Robotics with 3 complete chapters (Weeks 1-2, 3, 4). The textbook provides hands-on learning with runnable ROS 2 code examples, Mermaid diagrams, exercises, and mobile-responsive design. Content is structured for future RAG chatbot integration (Phase 2) with clean markdown format optimized for semantic chunking. Deployment via GitHub Actions to GitHub Pages with automated CI/CD pipeline ensuring quality gates (link checking, accessibility audits, build validation) before each release.

## Technical Context

**Language/Version**: TypeScript 5.7.2, JavaScript ES2022 (React/Docusaurus), Markdown/MDX v3
**Primary Dependencies**: Docusaurus 3.6.3, React 18.2.0, @docusaurus/theme-mermaid, Prism.js
**Storage**: Static files (markdown), no database (GitHub repository as source of truth)
**Testing**: Lighthouse CI (accessibility/performance), broken-link-checker, markdownlint, manual ROS 2 code validation
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge last 2 years), mobile-responsive (360px+ width)
**Project Type**: Web documentation site (static SSG - Static Site Generator)
**Performance Goals**:
- First Contentful Paint <1.5s
- Time to Interactive <3s
- Lighthouse Performance 95+ (mobile), 98+ (desktop)
- Page load <2s on 4G connection
**Constraints**:
- GitHub Pages free tier (1GB storage, 100GB bandwidth/month)
- Static hosting only (no server-side processing)
- Build timeout 10 minutes max
- Bundle size <150KB JavaScript initial load
**Scale/Scope**:
- Initial: 3 chapters (~30 content pages)
- Future: 13 weeks (estimated 130 content pages)
- Target audience: 1000-5000 students annually
- Estimated traffic: 10K-50K page views/month

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Educational Excellence & Accessibility ✅

**Requirements**:
- Interactive, personalized learning with runnable examples and exercises
- Content accuracy and pedagogical soundness
- Accessible design (Urdu translation in Phase 5)

**Compliance**:
- ✅ All chapters include hands-on exercises with clear success criteria
- ✅ Code examples tested in ROS 2 Humble environment before publication
- ✅ WCAG 2.1 Level AA compliance target (Lighthouse score 95+)
- ✅ Content validation via technical-reviewer and ros2-expert agents
- ✅ Progressive difficulty levels (beginner → intermediate → advanced)
- ⏳ Urdu translation planned for Phase 5

**Agent/Skill Utilization**:
- educational-content-designer: Chapter structure planning
- lesson-writer: Content drafting
- ros2-expert: Code validation
- technical-reviewer: Accuracy verification
- robotics-curriculum-designer: Pedagogy guidance

### II. Robust Technical Architecture ✅

**Requirements**:
- Frontend: React functional components, hooks, TypeScript, accessibility
- Adherence to defined standards

**Compliance**:
- ✅ Docusaurus 3.6.3 built on React 18 with TypeScript
- ✅ Functional components throughout (no class components)
- ✅ Hooks-based state management (React built-in)
- ✅ Semantic HTML5 structure for accessibility
- ✅ Mobile-first responsive design (360px+ support)
- ✅ Progressive enhancement approach

**Agent/Skill Utilization**:
- react-developer: Custom component development (if needed beyond Docusaurus defaults)

### III. Secure & Private Data Handling ✅

**Requirements**:
- Secure handling of API keys and user information
- Environment variables for secrets

**Compliance**:
- ✅ No backend in Phase 1 (static site = no user data collection)
- ✅ No authentication required (public educational content)
- ✅ No API keys stored in repository
- ⏳ Phase 2 (RAG chatbot): OpenAI API key via environment variables (.env)
- ⏳ Phase 3: Authentication with JWT, bcrypt password hashing

**Note**: Static site architecture inherently secure (no server, no database, no user data).

### IV. Performance & Scalability ✅

**Requirements**:
- Page load <2s
- Mobile responsive
- Optimization for free tier limits

**Compliance**:
- ✅ Target: First Contentful Paint <1.5s, Time to Interactive <3s
- ✅ Code splitting (automatic via Docusaurus)
- ✅ Lazy loading for images and Mermaid diagrams
- ✅ WebP image format for 60-80% size reduction
- ✅ Bundle size monitoring (<150KB JavaScript)
- ✅ GitHub Pages free tier sufficient (100GB bandwidth >> expected 10-50K page views/month)
- ✅ CDN via GitHub Pages (global distribution)

### V. Operational Readiness & DevOps ✅

**Requirements**:
- CI/CD via GitHub Actions
- Observability, alerting, deployment/rollback strategies

**Compliance**:
- ✅ GitHub Actions workflow for automated build and deploy
- ✅ Validation pipeline: markdown linting, link checking, Lighthouse audit
- ✅ Build verification before deployment
- ✅ Automated deployment to GitHub Pages on merge to main
- ✅ Rollback: Revert commit or deploy previous build from GitHub Pages history
- ✅ Build logs available in GitHub Actions interface
- ⏳ Advanced monitoring in Phase 2 (backend metrics, chatbot response times)

### VI. Collaborative & Agile Project Management ✅

**Requirements**:
- Conventional commits
- Atomic changes with descriptive messages
- Task tracking

**Compliance**:
- ✅ Conventional commit format: `docs(week-03): add ROS 2 publisher example`
- ✅ Atomic commits per section/feature
- ✅ Task tracking via GitHub Issues/Projects
- ✅ Pull request reviews before merge
- ✅ Branch naming: `content/week-XX-section-name`

### Risk Management & ADR Policy ✅

**Identified Risks** (detailed in research.md):
1. Content quality inconsistency → Mitigation: Agent validation chain
2. ROS 2 code examples fail → Mitigation: Mandatory testing in Docker container
3. Free tier limits exceeded → Mitigation: Monitoring + Cloudflare fallback
4. Search not working → Mitigation: Algolia + client-side fallback
5. Mobile performance below target → Mitigation: Lighthouse CI blocking merge

**ADR Candidates**:
Three architecturally significant decisions identified (pending user approval):
1. **Docusaurus over custom Next.js** - Framework choice with long-term impact
2. **TypeScript over JavaScript** - Type safety affecting all code
3. **Incremental content (3 chapters first)** - Delivery strategy affecting timeline

**Action**: After plan approval, suggest documenting these via `/sp.adr` command.

### **Constitution Compliance: PASS** ✅

All requirements satisfied or have clear path to compliance in future phases. No violations requiring justification. Ready to proceed with implementation.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
physical-ai-textbook/                    # Repository root
├── .github/
│   └── workflows/
│       ├── deploy.yml               # CI/CD: Build & deploy to GitHub Pages
│       └── validate.yml             # CI/CD: Lint, link check, Lighthouse
│
├── docs/                            # ALL TEXTBOOK CONTENT (Markdown/MDX)
│   ├── intro.md                     # Landing page
│   │
│   ├── week-01-02-intro-physical-ai/
│   │   ├── category.json            # Sidebar: {label, position, link}
│   │   ├── index.md                 # Chapter overview + learning objectives
│   │   ├── 01-foundations.md        # Section 1
│   │   ├── 02-embodied-intelligence.md
│   │   ├── 03-sensor-systems.md
│   │   ├── 04-physical-vs-digital-ai.md
│   │   └── exercises.md             # Hands-on practice
│   │
│   ├── week-03-ros2-part1/
│   │   ├── category.json
│   │   ├── index.md
│   │   ├── 01-ros2-architecture.md
│   │   ├── 02-nodes-topics.md
│   │   ├── 03-first-python-node.md
│   │   ├── 04-publisher-subscriber.md
│   │   └── exercises.md
│   │
│   ├── week-04-ros2-part2/
│   │   ├── category.json
│   │   ├── index.md
│   │   ├── 01-services-actions.md
│   │   ├── 02-simple-pub-sub.md
│   │   ├── 03-message-types.md
│   │   ├── 04-launch-files.md
│   │   └── exercises.md
│   │
│   └── resources/
│       ├── glossary.md              # Alphabetical term definitions
│       └── setup-guide.md           # ROS 2 installation instructions
│
├── src/                             # React components & custom code
│   ├── components/
│   │   ├── HomepageFeatures/
│   │   │   ├── index.tsx            # Feature cards on homepage
│   │   │   └── styles.module.css
│   │   └── CodeBlock/               # Custom code display (if needed)
│   │       └── index.tsx
│   │
│   ├── css/
│   │   └── custom.css               # Theme customization (colors, fonts)
│   │
│   └── pages/
│       ├── index.tsx                # Homepage React component
│       └── index.module.css
│
├── static/                          # Static assets (not processed by webpack)
│   ├── img/
│   │   ├── logo.svg                 # Site logo
│   │   ├── favicon.ico
│   │   ├── social-card.png          # Open Graph image
│   │   └── diagrams/                # Custom illustrations (future)
│   └── robots.txt
│
├── build/                           # Generated by npm run build (gitignored)
│   └── [production static files]
│
├── docusaurus.config.ts             # Main Docusaurus configuration
├── sidebars.ts                      # Sidebar navigation structure
├── package.json                     # Dependencies and scripts
├── package-lock.json
├── tsconfig.json                    # TypeScript configuration
├── .gitignore
├── .prettierrc                      # Code formatting rules
├── .markdownlint.json               # Markdown linting rules
└── README.md                        # Project documentation

companion-repo: physical-ai-code-examples/  # Separate GitHub repository
├── week-01-02-physical-ai/          # (Conceptual chapter, no code)
├── week-03-ros2-part1/
│   ├── 01-simple-publisher/
│   │   ├── simple_publisher.py      # Complete runnable code
│   │   ├── package.xml              # ROS 2 package manifest
│   │   ├── setup.py
│   │   ├── README.md                # Setup instructions
│   │   └── test_simple_publisher.py
│   ├── 02-pub-sub-pattern/
│   └── ...
├── week-04-ros2-part2/
│   └── ...
├── .github/workflows/
│   └── test-ros2-examples.yml       # CI/CD: Test all code in ROS 2 Docker
└── README.md
```

**Structure Decision**:

**Primary Repository** (`physical-ai-textbook`):
- **Selected**: Web documentation site (static SSG)
- **Rationale**: Docusaurus generates static HTML/CSS/JS from markdown, no backend needed
- **Content**: All educational content in `docs/` directory as markdown files
- **Code**: Minimal React components in `src/` for customization (homepage, theme)
- **Deployment**: `build/` directory deployed to GitHub Pages

**Companion Repository** (`physical-ai-code-examples`):
- **Purpose**: Complete, runnable ROS 2 code examples separate from textbook
- **Rationale**: Enables students to clone/fork working code independently
- **Testing**: CI/CD pipeline tests all code in ROS 2 Docker container
- **Integration**: Textbook links to specific code files in companion repo

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations** - Constitution Check passed completely. No complexity justifications required.

## Implementation Phases

### Phase 0: Research & Technical Decisions ✅ COMPLETE

**Deliverable**: `research.md` (17 decisions documented)

**Key Decisions Made**:
1. Framework: Docusaurus 3.6.3 over Next.js/VuePress
2. Search: Algolia DocSearch (free tier)
3. Diagrams: Mermaid.js for architecture, text placeholders for custom illustrations
4. Deployment: GitHub Actions → GitHub Pages
5. Code highlighting: Prism.js with ROS 2 language extensions
6. Content workflow: Agent-driven (educational-content-designer → lesson-writer → technical-reviewer)
7. Mobile strategy: Mobile-first responsive design (360px+ target)
8. Performance: Aggressive code splitting + lazy loading
9. Accessibility: WCAG 2.1 AA compliance, Lighthouse CI enforcement
10. Companion repository: Separate repo for runnable ROS 2 code examples

**All NEEDS CLARIFICATION items resolved** - Ready for Phase 1.

### Phase 1: Design & Contracts ✅ COMPLETE

**Deliverables**:
- ✅ `data-model.md` - 9 content entities defined with validation rules
- ✅ `contracts/chatbot-api.yaml` - OpenAPI 3.1 spec for Phase 2 RAG chatbot
- ✅ `contracts/README.md` - API documentation and integration guides
- ✅ `quickstart.md` - Contributor setup guide

**Data Model Entities**:
1. Chapter (learning module for 1-2 weeks)
2. Section (focused topic page)
3. Code Example (runnable ROS 2 code snippet)
4. Exercise (hands-on practice activity)
5. Diagram (Mermaid.js visualizations)
6. Cross-Reference (inter-section links)
7. Glossary Term (domain-specific definitions)
8. Navigation Item (sidebar menu structure)
9. Theme Preference (light/dark mode state)

**API Contracts** (for Phase 2):
- POST /chat - Send question, receive answer with citations
- POST /chat/stream - Streaming chatbot response (SSE)
- GET /content/search - Semantic search across textbook
- GET /content/sections/{id} - Retrieve specific section
- GET /content/related - Find related sections

### Phase 2: Implementation Tasks ⏳ NEXT STEP

**Command**: `/sp.tasks` (generates `tasks.md`)

**Scope**: Generate dependency-ordered, testable implementation tasks for:
1. Project initialization (npm create docusaurus@latest)
2. Configuration setup (docusaurus.config.ts, sidebars.ts)
3. Theme customization (colors, fonts, layout)
4. Content structure creation (3 chapters × 5-6 sections each)
5. Custom components (if needed beyond Docusaurus defaults)
6. CI/CD pipeline setup (GitHub Actions workflows)
7. Deployment to GitHub Pages
8. Quality validation (Lighthouse, link checking, accessibility)

**Expected Tasks File Output**: 30-50 atomic, testable tasks with:
- Clear acceptance criteria
- Code references where applicable
- Dependencies explicitly stated
- Test cases for validation

### Phase 3: Execution (Future)

**Command**: `/sp.implement` (processes `tasks.md`)

Autonomous execution of all tasks from tasks.md with:
- Dependency resolution
- Automated testing
- Progress tracking
- Error handling and recovery

### Phase 4: Deployment Validation (Future)

**Post-Implementation Checks**:
- ✅ All Lighthouse scores meet targets (Performance 95+, Accessibility 95+)
- ✅ No broken links (internal or external)
- ✅ Mobile responsive (tested 360px, 768px, 1920px)
- ✅ Dark/light theme working consistently
- ✅ Search functional (Algolia or client-side fallback)
- ✅ All code examples tested in ROS 2 Humble
- ✅ CI/CD pipeline green (all checks passing)
- ✅ GitHub Pages deployment successful

## Architectural Decision Records (ADR)

**Recommendation**: Document the following decisions after user approval:

### ADR-001: Choose Docusaurus over Custom Next.js Application

**Context**: Need static site generator for educational content with built-in docs features.

**Decision**: Docusaurus 3.6.3

**Consequences**:
- ✅ Built-in search, versioning, i18n support
- ✅ Faster development (weeks vs months)
- ✅ Production-proven stability
- ✅ React-based for future chatbot widget integration
- ⚠️ Less flexible than custom Next.js (acceptable trade-off for this use case)

**Alternatives Considered**:
- Next.js + Nextra: More flexible but requires custom implementation of docs features
- VuePress: Strong but smaller ecosystem, less React integration
- GitBook: SaaS limitation, less control

**Status**: Accepted (pending formal ADR documentation)

---

### ADR-002: TypeScript over JavaScript

**Context**: Need type safety for maintainability as project grows to 130+ pages.

**Decision**: TypeScript 5.7.2

**Consequences**:
- ✅ Type safety catches errors early
- ✅ Better IDE support and autocomplete
- ✅ Easier refactoring as codebase evolves
- ✅ Self-documenting code through type definitions
- ⚠️ Slight learning curve for contributors unfamiliar with TypeScript

**Alternatives Considered**:
- JavaScript: Faster initial development but harder to maintain long-term
- JSDoc comments: Type hints without TypeScript overhead, but less robust

**Status**: Accepted (pending formal ADR documentation)

---

### ADR-003: Incremental Content Approach (3 Chapters First)

**Context**: Need to balance speed-to-market with quality and validate architecture early.

**Decision**: Build and deploy 3 chapters initially (Weeks 1-2, 3, 4), expand to 13 weeks iteratively.

**Consequences**:
- ✅ Early validation of structure and pedagogy
- ✅ Time to gather student feedback before full content creation
- ✅ Easier to prepare content for RAG chatbot (smaller corpus initially)
- ✅ Manageable scope for hackathon timeline
- ⚠️ Incomplete curriculum initially (mitigated by clear roadmap to full 13 weeks)

**Alternatives Considered**:
- Full 13 weeks upfront: Higher quality risk, longer time to first deployment
- Single chapter pilot: Too small to validate multi-chapter navigation and cross-references

**Status**: Accepted (pending formal ADR documentation)

---

**Action Required**: After plan approval, run `/sp.adr [decision-title]` for each ADR to formally document these decisions.

## Next Steps

1. **User reviews this plan** - Approval required before proceeding
2. **Document ADRs** (if approved) - Run `/sp.adr` for the 3 decisions above
3. **Generate tasks** - Run `/sp.tasks` to create dependency-ordered implementation tasks
4. **Begin implementation** - Either manual execution or `/sp.implement` for autonomous task processing

## References

**Generated Artifacts** (this planning session):
- [research.md](./research.md) - Technical decisions and stack justification
- [data-model.md](./data-model.md) - Content entity definitions and validation
- [contracts/chatbot-api.yaml](./contracts/chatbot-api.yaml) - API specification for Phase 2
- [contracts/README.md](./contracts/README.md) - API integration documentation
- [quickstart.md](./quickstart.md) - Contributor setup guide

**Source Documents**:
- [spec.md](./spec.md) - Feature specification with user stories and requirements
- [.specify/memory/constitution.md](../../.specify/memory/constitution.md) - Project principles and standards

**External Resources**:
- [Docusaurus Documentation](https://docusaurus.io/docs)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MDX Specification](https://mdxjs.com/)
- [Mermaid Diagram Syntax](https://mermaid.js.org/)
- [WCAG 2.1 Level AA](https://www.w3.org/WAI/WCAG21/quickref/?currentsidebar=%23col_customize&levels=aaa)
