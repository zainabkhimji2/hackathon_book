# Architectural Plan: Docusaurus Skeleton for Physical AI & Humanoid Robotics Textbook

## 1. Scope and Dependencies:

### In Scope:
- Docusaurus 3.x project initialization with React 18.x and TypeScript.
- Implementation of 3 core chapters as outlined in the content architecture.
- Custom Docusaurus CSS and color palette integration.
- MDX content format with Mermaid diagrams and Prism highlighting (including ROS 2 language support).
- Project structure as specified.
- Docusaurus configuration (title, tagline, URL, baseUrl, orgName, projName, deploymentBranch, trailingSlash).
- Theme configuration for navbar, footer, prism, algolia (setup later), and colorMode.
- Core Docusaurus plugins and theme-mermaid plugin.
- Detailed chapter structure including `index.md`, content files, and `exercises.md`.
- Content frontmatter with title, sidebar_label, sidebar_position, description, and keywords.
- Standardized code block pattern with title, line numbers, and syntax highlighting.
- Sidebar navigation structure.
- Breadcrumb and pagination implementation.
- GitHub Actions workflow for deployment to gh-pages branch on push to main.
- GitHub Pages configuration (source branch, optional custom domain, HTTPS enforced).
- Basic performance optimizations (code splitting, WebP for images, lazy loading, minification).
- Adherence to accessibility architecture principles (semantic HTML, ARIA, keyboard navigation, screen reader support, color contrast, focus indicators, alt text).
- Content preparation for future RAG ingestion (clean markdown, consistent headings, chunk-friendly structure, metadata).
- Risk mitigation strategies as outlined.
- Definition of Done checklist implementation.
- Outlining follow-up phases for future development.

### Out of Scope:
- Full 13-week content creation in this phase.
- Algolia search configuration (will be set up later).
- Custom plugin for chatbot integration (future).
- Custom domain setup for GitHub Pages (optional for later).
- Service worker caching strategy (future).
- RAG Chatbot Integration (Phase 2).
- Authentication & User Profiling (Phase 3).
- Content Personalization (Phase 4).
- Urdu Translation (Phase 5).

### External Dependencies:
- GitHub (for source control and GitHub Pages deployment).
- Node.js (for Docusaurus build process).
- npm (for package management).

## 2. Key Decisions and Rationale:

- **Frontend Framework: Docusaurus 3.x with React 18.x and TypeScript**
  - **Options Considered:** Custom Next.js app, Gatsby.
  - **Trade-offs:** Custom Next.js offers more flexibility but increases development time and maintenance. Gatsby has a steeper learning curve for non-React developers.
  - **Rationale:** Docusaurus is a production-ready docs platform with built-in features (search, versioning, i18n ready), excellent performance, and is React-based for future chatbot integration. TypeScript provides type safety and better IDE support.

- **Styling: Docusaurus custom CSS + CSS modules (No Tailwind CSS)**
  - **Options Considered:** Tailwind CSS.
  - **Trade-offs:** Tailwind offers utility-first styling but introduces an additional framework and potentially larger CSS bundles if not purged effectively.
  - **Rationale:** Sticking to the Docusaurus theming system and custom CSS reduces complexity and ensures consistency with the Docusaurus ecosystem. Custom color palette for branding.

- **Content Format: MDX (Markdown + JSX components) with Mermaid and Prism**
  - **Options Considered:** Pure Markdown.
  - **Trade-offs:** Pure Markdown is simpler but lacks the extensibility of MDX for interactive components.
  - **Rationale:** MDX allows for rich, interactive content. Mermaid provides powerful diagramming capabilities, and Prism ensures high-quality code highlighting with specific support for ROS 2 languages.

- **Incremental Content Approach (3 chapters first)**
  - **Options Considered:** Developing all 13 weeks of content upfront.
  - **Trade-offs:** Developing all content at once is a larger initial effort and delays feedback.
  - **Rationale:** Starting with 3 chapters allows for validating the structure, preparing for RAG, and maintaining a manageable scope for Phase 1. This enables agile iteration and early user feedback.

## 3. Interfaces and API Contracts:

- **Public APIs:** Not applicable in this phase (static documentation site).
- **Versioning Strategy:** Docusaurus's built-in versioning will be utilized for future content updates, though not explicitly configured in Phase 1.
- **Idempotency, Timeouts, Retries:** Not applicable in this phase.
- **Error Taxonomy:** Not applicable in this phase.

## 4. Non-Functional Requirements (NFRs) and Budgets:

- **Performance:**
  - First Contentful Paint: <1.5s
  - Time to Interactive: <3s
  - Lighthouse Performance: 95+
- **Reliability:** GitHub Pages provides high availability.
- **Security:** Standard web security best practices for a static site (HTTPS enforced via GitHub Pages).
- **Cost:** Minimal, leveraging free GitHub Pages hosting.

## 5. Data Management and Migration:

- **Source of Truth:** Markdown/MDX files within the `docs/` directory.
- **Schema Evolution:** N/A for static content, but content frontmatter will be consistently structured.
- **Migration and Rollback:** Git provides version control for content and configuration. GitHub Pages deployment allows rolling back to previous successful deployments.
- **Data Retention:** All content is stored in the Git repository.

## 6. Operational Readiness:

- **Observability:** GitHub Actions logs for deployment status. Docusaurus build output for errors.
- **Alerting:** GitHub Actions workflow failure notifications.
- **Runbooks:** Basic setup instructions in `README.md`.
- **Deployment and Rollback strategies:** Automated via GitHub Actions to `gh-pages` branch. Manual rollback by reverting `main` branch commits.
- **Feature Flags and compatibility:** Not applicable in this phase.

## 7. Risk Analysis and Mitigation:

- **Risk 1: Content Quality Inconsistency**
  - **Blast Radius:** Poor learning experience, reduced credibility.
  - **Mitigation:** Enforce a structured content creation workflow using specified subagents (educational-content-designer, lesson-writer) and skills (learning-objectives, code-example-generator, exercise-designer, technical-clarity). Implement final validation with the `technical-reviewer` subagent.

- **Risk 2: Navigation Complexity**
  - **Blast Radius:** User frustration, difficulty finding information.
  - **Mitigation:** Start with a flat hierarchy for the initial 3 chapters. Test navigation with potential users for usability and iteratively refine the sidebar and breadcrumb structures based on feedback.

- **Risk 3: ROS 2 Code Examples Don't Work**
  - **Blast Radius:** User frustration, inability to follow exercises, damaged credibility.
  - **Mitigation:** All ROS 2 code examples will be rigorously tested in a ROS 2 Humble environment before publication. Utilize the `ros2-expert` subagent for validation during the content creation workflow.

## 8. Evaluation and Validation:

- **Definition of Done (tests, scans):** (See "DEFINITION OF DONE" in prompt - replicated in section 9 for clarity)
- **Output Validation for format/requirements/safety:**
  - Docusaurus build success.
  - Visual inspection for correct rendering of MDX, Mermaid, and code blocks.
  - Manual checks for navigation, responsiveness, and accessibility.
  - Lighthouse audits for performance, accessibility, best practices, and SEO scores.
  - No console errors or warnings in the browser.

## 9. Architectural Decision Record (ADR):

**Architectural decisions detected (to be documented):**
1. Choice of Docusaurus over custom Next.js app.
2. TypeScript over JavaScript.
3. Incremental content approach (3 chapters first).

---

## DEFINITION OF DONE:

- [ ] Docusaurus project initialized with TypeScript
- [ ] All 3 chapters written with complete content
- [ ] All code examples tested and validated
- [ ] Navigation works smoothly (sidebar, breadcrumbs, prev/next)
- [ ] Dark/light theme working
- [ ] Mobile responsive (tested on 360px)
- [ ] Search functionality configured
- [ ] GitHub Actions deployment configured
- [ ] Deployed successfully to GitHub Pages
- [ ] Lighthouse scores: Performance 95+, Accessibility 95+
- [ ] No console errors or warnings
- [ ] `README.md` with setup instructions

## FOLLOW-UP PHASES:

**After Phase 1 completion:**
- **Phase 2:** RAG Chatbot Integration (using `/subagent api-architect` for planning)
- **Phase 3:** Authentication & User Profiling
- **Phase 4:** Content Personalization
- **Phase 5:** Urdu Translation
- **Phase 6:** Expand to 13 weeks of content

**Recommended Subagents/Skills for Future Phases:**
- `/subagent react-developer` for custom component development.
- `/subagent api-architect` when planning RAG integration architecture.
- `/skill github-actions` for CI/CD pipeline setup.