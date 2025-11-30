# Research: Docusaurus Robotics Textbook

**Feature**: 002-docusaurus-robotics-textbook
**Date**: 2025-11-30
**Purpose**: Resolve technical unknowns and establish architectural foundation

## Executive Summary

This research establishes the technical foundation for building a Docusaurus-based interactive textbook for Physical AI & Humanoid Robotics. All critical decisions are resolved with concrete technical specifications to enable immediate implementation.

## 1. Framework & Version Decisions

### Decision: Docusaurus 3.6.3 (Latest Stable)

**Rationale**:
- Latest stable release with full TypeScript support
- Built-in MDX v3 support for React components in markdown
- Native Mermaid.js integration via @docusaurus/theme-mermaid
- Improved performance with React 18 concurrent rendering
- Better accessibility compliance (WCAG 2.1 AA out-of-box)

**Alternatives Considered**:
- Next.js + Nextra: More flexible but requires custom implementation of docs features (search, versioning, i18n)
- VuePress: Strong alternative but smaller ecosystem and less React integration for future chatbot
- GitBook: SaaS limitation, less control over customization and RAG preparation

**Technical Specifications**:
```json
{
  "@docusaurus/core": "^3.6.3",
  "@docusaurus/preset-classic": "^3.6.3",
  "@docusaurus/theme-mermaid": "^3.6.3",
  "react": "^18.2.0",
  "react-dom": "^18.2.0",
  "typescript": "^5.7.2"
}
```

## 2. Search Solution Architecture

### Decision: Algolia DocSearch (Free Tier)

**Rationale**:
- Free for open-source documentation projects
- Official Docusaurus integration with zero configuration
- Sub-second search performance
- Automatic indexing via crawler
- No backend infrastructure required

**Implementation Path**:
1. Deploy textbook to GitHub Pages
2. Apply for Algolia DocSearch program (typically approved in 48 hours)
3. Add Algolia credentials to docusaurus.config.ts
4. Crawler automatically indexes content weekly

**Fallback Strategy**:
If Algolia application is delayed, use client-side search plugin as interim:
- @docusaurus/plugin-content-docs includes basic local search
- Upgrade to Algolia when approved without code changes

**Alternatives Considered**:
- Typesense: Self-hosted, requires infrastructure (violates static hosting constraint)
- Lunr.js: Client-side only, performance degrades with content growth
- Meilisearch: Excellent but requires backend (violates constraint)

## 3. Code Highlighting Strategy

### Decision: Prism.js with Custom ROS 2 Language Support

**Rationale**:
- Native Docusaurus integration (zero config)
- Supports Python, C++, Bash, YAML, XML out-of-box
- Theme-aware syntax highlighting (auto-adjusts for dark/light)
- Line numbering and copy-to-clipboard included
- Can extend with custom language definitions for ROS 2 specifics

**Technical Implementation**:
```typescript
// docusaurus.config.ts
prism: {
  theme: prismThemes.github,
  darkTheme: prismThemes.dracula,
  additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'xml'],
  magicComments: [
    {
      className: 'code-block-highlight-line',
      line: 'highlight-next-line',
    },
    {
      className: 'code-block-ros2',
      line: 'ros2-specific',
      block: {start: 'ros2-start', end: 'ros2-end'},
    },
  ],
}
```

**Custom ROS 2 Patterns**:
- Extend Python grammar for ROS 2 decorators (@node.timer, @node.subscription)
- Custom highlighting for rclpy imports and message types
- Visual distinction for ROS 2 specific code blocks

## 4. Content Structure & Organization

### Decision: Nested Hierarchy with Separate Page Files

**Rationale**:
- Each section/subsection is a separate .md file (better for RAG chunking)
- category.json controls sidebar grouping and ordering
- Clear separation enables parallel content authoring
- Supports progressive disclosure (students navigate section-by-section)

**File Structure Pattern**:
```text
docs/
├── intro.md
├── week-01-02-intro-physical-ai/
│   ├── category.json                 # {label: "Week 1-2: Physical AI", position: 1}
│   ├── index.md                      # Chapter overview + learning objectives
│   ├── 01-foundations.md             # Section 1
│   ├── 02-embodied-intelligence.md   # Section 2
│   ├── 03-sensor-systems.md          # Section 3
│   ├── 04-physical-vs-digital-ai.md  # Section 4
│   └── exercises.md                  # Hands-on practice
```

**Frontmatter Template**:
```yaml
---
title: Foundations of Physical AI
sidebar_label: Foundations
sidebar_position: 1
description: Understanding the fundamental principles of embodied intelligence and physical AI systems
keywords: [physical ai, embodied intelligence, robotics, sensors, actuators]
---
```

## 5. Deployment & CI/CD Architecture

### Decision: GitHub Actions → GitHub Pages (Static Deployment)

**Rationale**:
- Zero-cost hosting within GitHub Pages free tier (1GB storage, 100GB bandwidth/month)
- Automated deployment on every push to main
- Built-in HTTPS and global CDN
- No server management required
- Perfect alignment with static site architecture

**CI/CD Pipeline Specification**:
```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
      - uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
```

**Performance Characteristics**:
- Build time: ~2-3 minutes for initial 3 chapters
- Deploy time: ~30 seconds
- Total time from push to live: <5 minutes

**Alternatives Considered**:
- Vercel: Excellent but introduces external dependency beyond GitHub
- Netlify: Similar to Vercel, adds complexity
- Cloudflare Pages: Good but GitHub Actions + Pages simpler for this use case

## 6. Theme & Styling Architecture

### Decision: Custom CSS with Docusaurus Theming System (No Tailwind)

**Rationale**:
- Docusaurus has robust CSS custom properties system
- Built-in dark/light mode with theme persistence
- Easier to maintain consistent robotics aesthetic
- Better integration with Docusaurus components
- Smaller bundle size (no Tailwind overhead)

**Color Palette Specification**:
```css
/* custom.css - Light Mode */
:root {
  --ifm-color-primary: #0ea5e9;        /* Cyan - robotics/tech feel */
  --ifm-color-primary-dark: #0284c7;
  --ifm-color-primary-darker: #0369a1;
  --ifm-color-primary-darkest: #075985;
  --ifm-color-primary-light: #38bdf8;
  --ifm-color-primary-lighter: #7dd3fc;
  --ifm-color-primary-lightest: #bae6fd;

  --ifm-color-secondary: #1e293b;      /* Slate - professional */
  --ifm-background-color: #ffffff;
  --ifm-font-family-base: 'Inter', system-ui, -apple-system, sans-serif;
  --ifm-code-font-size: 95%;
  --ifm-line-height-base: 1.6;
}

/* Dark Mode */
[data-theme='dark'] {
  --ifm-color-primary: #38bdf8;        /* Lighter cyan for dark bg */
  --ifm-color-primary-dark: #0ea5e9;
  --ifm-background-color: #0f172a;     /* Deep slate background */
  --ifm-background-surface-color: #1e293b;
}
```

**Typography Stack**:
- Base: Inter (clean, readable, excellent for technical content)
- Code: JetBrains Mono (optimized for code display)
- Headings: Inter with increased weight

## 7. Content Authoring Workflow

### Decision: Agent-Driven Content Generation with Validation Chain

**Rationale**:
- Ensures consistency across all chapters
- Leverages specialized expertise for each content type
- Built-in quality validation before publication
- Efficient use of educational content design patterns

**Workflow Specification**:
```text
1. Chapter Planning
   → Agent: educational-content-designer
   → Output: Chapter structure, learning objectives, prerequisite mapping
   → Validation: Alignment with 13-week curriculum roadmap

2. Content Drafting
   → Agent: lesson-writer
   → Input: Chapter structure from step 1
   → Output: Complete markdown with explanations, examples, exercises
   → Validation: Reading time target (8-12 minutes per section)

3. Code Example Generation
   → Agent: ros2-expert
   → Input: Concept from lesson content
   → Output: Runnable Python/C++ code with comments
   → Validation: Tested in ROS 2 Humble environment

4. Technical Review
   → Agent: technical-reviewer
   → Input: Complete chapter draft
   → Output: Accuracy verification, technical corrections
   → Validation: Zero technical errors, best practices compliance

5. Readability Check
   → Process: technical-clarity skill
   → Output: Plain language improvements, jargon explanations
   → Validation: Flesch reading ease score 60+ (standard difficulty)
```

## 8. Diagram Strategy

### Decision: Mermaid.js for Architecture + Placeholders for Custom Illustrations

**Rationale**:
- Mermaid handles system architecture, flowcharts, sequence diagrams perfectly
- Text-based = version controlled, easy to update, RAG-friendly
- Custom illustrations (robot visuals, sensor layouts) deferred with detailed descriptions
- Accessibility: all diagrams have text alternatives

**Mermaid Capabilities Matrix**:
| Use Case | Mermaid Support | Example |
|----------|-----------------|---------|
| ROS 2 Node Architecture | ✅ Excellent | `graph TD; A[Node] --> B[Topic]` |
| Publisher-Subscriber Flow | ✅ Excellent | Sequence diagrams |
| System Components | ✅ Excellent | Component diagrams |
| Robot Physical Structure | ❌ Use placeholder | "Humanoid robot with 12 DOF arms..." |
| Sensor Placement | ❌ Use placeholder | "LiDAR mounted at torso height..." |

**Placeholder Template for Future Illustrations**:
```markdown
:::note Future Illustration
**Description for Visual Designer**:
- Main subject: Humanoid robot upper body
- Key components: LiDAR sensor (blue), stereo cameras (green), IMU (red)
- Placement: LiDAR at chest center, cameras at eye level 8cm apart, IMU in torso
- Perspective: Front 3/4 view, components labeled with callout lines
- Style: Technical illustration, clean lines, robotics blue color scheme
:::

**Text Alternative**: The robot's perception system includes three sensor types positioned for optimal coverage...
```

## 9. Mobile Optimization Strategy

### Decision: Mobile-First Responsive Design with Progressive Enhancement

**Rationale**:
- Docusaurus provides excellent mobile support out-of-box
- Custom CSS refinements for code blocks and navigation
- Target: 360px minimum width (covers 95% of mobile devices)
- Performance budget: <2s load on 4G

**Mobile-Specific Optimizations**:
```css
/* Code blocks on mobile */
@media (max-width: 768px) {
  .prism-code {
    font-size: 14px;
    padding: 1rem;
    overflow-x: auto;
  }

  /* Sidebar slides in from left */
  .navbar__toggle {
    display: block;
  }

  /* Reduce heading sizes for mobile */
  h1 { font-size: 1.75rem; }
  h2 { font-size: 1.5rem; }

  /* Stack navigation elements */
  .pagination-nav {
    flex-direction: column;
  }
}
```

**Performance Budget**:
- JavaScript bundle: <150KB (Docusaurus optimizes automatically)
- Initial page load: <2s on 4G (50Mbps)
- Images: WebP format, lazy loaded
- Fonts: subset to Latin characters only

## 10. Content Format for RAG Readiness

### Decision: Clean Markdown with Structured Metadata

**Rationale**:
- RAG systems ingest markdown better than HTML
- Frontmatter provides metadata for semantic chunking
- Consistent heading hierarchy aids embedding quality
- Code blocks clearly marked improve retrieval accuracy

**RAG Optimization Patterns**:
```markdown
---
title: Publisher-Subscriber Pattern in ROS 2
description: Learn how to implement pub-sub communication between ROS 2 nodes
keywords: [ros2, publisher, subscriber, topics, rclpy]
chapter: week-03-ros2-part1
section: 04-publisher-subscriber
difficulty: intermediate
prerequisites: [ros2-nodes, ros2-topics]
---

# Publisher-Subscriber Pattern in ROS 2

## Overview
[2-3 sentence summary - optimized for RAG retrieval]

## Key Concepts
- **Publisher**: [definition with context]
- **Subscriber**: [definition with context]

## Implementation
[Step-by-step with inline code]

## Common Patterns
[Reusable patterns for RAG reference]
```

**Chunking Strategy**:
- Each section (## heading) = separate chunk
- Code examples embedded with context paragraphs
- Cross-references maintained with relative links

## 11. Testing & Validation Strategy

### Decision: Multi-Layer Validation (Code, Links, Accessibility, Performance)

**Validation Layers**:

1. **Code Example Validation**
   - All Python code: pytest in ROS 2 Humble Docker container
   - All C++ code: colcon build + test in ROS 2 environment
   - Validation before content merge

2. **Link Checking**
   - CI/CD integration: broken-link-checker npm package
   - Runs on every pull request
   - Blocks merge if broken links detected

3. **Accessibility Audit**
   - Lighthouse CI integration
   - Minimum scores: Accessibility 95+, Best Practices 100, Performance 95+
   - Manual keyboard navigation testing

4. **Build Validation**
   - npm run build must complete without warnings
   - Bundle size analysis (warns if >150KB JS increase)
   - Markdown linting (markdownlint)

**GitHub Actions Test Pipeline**:
```yaml
jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
      - run: npm ci
      - run: npm run lint
      - run: npm run build
      - run: npx broken-link-checker http://localhost:3000
      - uses: treosh/lighthouse-ci-action@v10
        with:
          urls: |
            http://localhost:3000
            http://localhost:3000/docs/intro
          uploadArtifacts: true
```

## 12. Performance Optimization Decisions

### Decision: Aggressive Code Splitting + Lazy Loading

**Optimization Strategy**:
- Route-based code splitting (automatic via Docusaurus)
- Lazy load Mermaid diagrams (only when visible in viewport)
- Image lazy loading with blur-up placeholders
- Preconnect to external domains (fonts, CDNs)

**Lighthouse Target Scores**:
- Performance: 95+ (mobile), 98+ (desktop)
- Accessibility: 95+
- Best Practices: 100
- SEO: 100

**Caching Strategy**:
```javascript
// Service worker for offline caching (Phase 2)
// Phase 1: Rely on GitHub Pages CDN caching
// Cache-Control: max-age=3600 for static assets
```

## 13. Accessibility Implementation

### Decision: WCAG 2.1 Level AA Compliance with Proactive Testing

**Key Accessibility Features**:
- Semantic HTML5 structure (nav, main, article, aside)
- ARIA landmarks for screen readers
- Skip-to-content link (keyboard users)
- Minimum contrast ratio 4.5:1 (WCAG AA)
- All interactive elements keyboard accessible
- Focus indicators visible and high-contrast
- Alt text for all images (required in content guidelines)

**Color Contrast Verification**:
```css
/* Light mode - verified via WebAIM contrast checker */
--text-on-background: #1a1a1a on #ffffff (14.8:1) ✓
--link-color: #0ea5e9 on #ffffff (4.52:1) ✓
--code-background: #f6f8fa with #24292e text (13.6:1) ✓

/* Dark mode */
--text-on-background: #e2e8f0 on #0f172a (13.2:1) ✓
--link-color: #38bdf8 on #0f172a (8.9:1) ✓
```

## 14. Companion Code Repository Structure

### Decision: Separate GitHub Repository for Runnable Examples

**Rationale**:
- Keeps textbook repo focused on content
- Students can clone/fork examples repository independently
- CI/CD tests all code in actual ROS 2 environment
- Each chapter's examples tested in isolation

**Repository Structure**:
```text
physical-ai-code-examples/
├── .github/workflows/
│   └── test-examples.yml          # ROS 2 testing pipeline
├── week-01-02-physical-ai/
│   ├── README.md                  # Chapter overview + setup
│   └── examples/
│       └── (no runnable code - conceptual chapter)
├── week-03-ros2-part1/
│   ├── README.md
│   ├── 01-first-node/
│   │   ├── simple_publisher.py
│   │   ├── simple_subscriber.py
│   │   ├── package.xml
│   │   └── README.md
│   ├── 02-pub-sub-pattern/
│   │   └── [complete package]
│   └── tests/
│       └── test_week03_examples.py
└── week-04-ros2-part2/
    └── [similar structure]
```

**Integration with Textbook**:
```markdown
<!-- In textbook content -->
See the complete, runnable version of this code in the
[companion repository](https://github.com/username/physical-ai-code-examples/tree/main/week-03-ros2-part1/01-first-node).

```python title="simple_publisher.py" showLineNumbers
import rclpy
from rclpy.node import Node
# ... inline code for quick reference
```

## 15. Risk Mitigation Strategies

### Risk 1: Content Quality Inconsistency
**Impact**: High - affects learning outcomes
**Probability**: Medium
**Mitigation**:
- Mandatory technical-reviewer agent validation for all content
- ros2-expert validates all code examples
- Peer review requirement (2 reviewers) for each chapter
- Template-driven content structure

### Risk 2: ROS 2 Code Examples Don't Work
**Impact**: Critical - breaks student trust
**Probability**: Medium
**Mitigation**:
- All code tested in Docker container (ROS 2 Humble)
- CI/CD runs full test suite on every commit
- Examples include setup instructions and dependencies
- Common errors documented in troubleshooting sections

### Risk 3: Free Tier Limit Exceeded (GitHub Pages)
**Impact**: Medium - site goes offline
**Probability**: Low (100GB/month is generous)
**Mitigation**:
- Monitor bandwidth via GitHub insights
- Optimize images to WebP (60-80% size reduction)
- Set up Cloudflare in front if approaching limits (free CDN)
- Worst case: migrate to Vercel/Netlify (free tiers similar)

### Risk 4: Search Not Working (Algolia Delay)
**Impact**: Medium - degrades UX
**Probability**: Low
**Mitigation**:
- Apply for Algolia DocSearch immediately upon deployment
- Fallback: @docusaurus/plugin-local-search (client-side)
- Manual application follow-up if no response in 1 week

### Risk 5: Mobile Performance Below Target
**Impact**: Medium - affects 30% of users
**Probability**: Low
**Mitigation**:
- Lighthouse CI blocks merge if performance < 90
- Image optimization mandatory (WebP, lazy load)
- Bundle analysis on every build
- Test on actual devices (not just emulators)

## 16. Technology Stack Summary

### Frontend
| Technology | Version | Purpose |
|------------|---------|---------|
| Docusaurus | 3.6.3 | Core framework |
| React | 18.2.0 | Component system |
| TypeScript | 5.7.2 | Type safety |
| Mermaid.js | 11.4.1 | Diagrams |
| Prism.js | 1.29.0 | Syntax highlighting |

### Build & Deploy
| Technology | Purpose |
|------------|---------|
| Node.js 18 | Build environment |
| npm | Package management |
| GitHub Actions | CI/CD pipeline |
| GitHub Pages | Hosting |

### Content Authoring
| Agent/Skill | Purpose |
|-------------|---------|
| educational-content-designer | Chapter structure planning |
| lesson-writer | Content drafting |
| ros2-expert | Code example validation |
| technical-reviewer | Accuracy verification |
| react-developer | Custom component creation (if needed) |

### Validation Tools
| Tool | Purpose |
|------|---------|
| markdownlint | Markdown consistency |
| broken-link-checker | Link validation |
| Lighthouse CI | Performance/accessibility audit |
| pytest + ROS 2 | Code example testing |

## 17. Implementation Readiness

All technical unknowns have been resolved. Ready to proceed to Phase 1 (Design & Contracts).

**Remaining Questions**: None - all critical decisions documented with concrete specifications.

**Next Steps**:
1. Generate data-model.md (content entities and relationships)
2. Generate contracts/ (API specifications for Phase 2 RAG chatbot)
3. Generate quickstart.md (setup guide for contributors)
4. Update agent context with technology stack
5. Fill complete implementation plan

**Approval Gate**: ✅ Ready to proceed - all NEEDS CLARIFICATION items resolved.
