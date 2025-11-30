---
description: Create CI/CD pipelines for automated testing, building, and deployment using GitHub Actions.
---

Create CI/CD pipelines for automated testing, building, and deployment using GitHub Actions.

WHAT IT DOES:
Sets up workflows for: build on push, run tests, deploy to GitHub Pages/Vercel, environment management.

WORKFLOW STRUCTURE:
```yaml
name: Deploy Docusaurus
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
      - run: npm install
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

USE FOR: Automated deployment, CI/CD setup, testing pipelines.
