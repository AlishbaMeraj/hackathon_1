---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: ayat malik
command: /sp.constitution
labels: [constitution,spec-driven,ai-book,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text

Project: Spec-Driven AI Book with Embedded RAG Chatbot

Purpose:
Author and publish a technical book using Spec-Kit Plus and Claude Code, deployed via Docusaurus to GitHub Pages, with an embedded RAG chatbot that answers questions grounded strictly in the book content.

Core Principles:
- Spec-driven, deterministic development
- No hallucinations; content must be grounded
- Clarity for developers (CS/AI background)
- Reproducible, modular architecture
- Ethical and privacy-aware AI usage

Book Standards:
- Tools: Spec-Kit Plus, Claude Code, Docusaurus
- Format: Markdown, GitHub Pages deployment
- Style: Clear, instructional, code-first
- Structure: Concepts → Architecture → Implementation → Deployment

RAG Chatbot Standards:
- Stack: OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud (Free Tier)
- Retrieval strictly from book content
- Selected-text–only answering when user provides a selection
- Source-grounded, non-creative responses
- Embedded within the Docusaurus site

Constraints:
- Free-tier services only
- Environment-based configuration
- Documented APIs and setup steps
- Secure key handling

Success Criteria:
- Book live on GitHub Pages
- RAG chatbot answers content-based queries correctly
- Selected-text queries answered exclusively from provided text
- Project runs end-to-end from a fresh clone
```

## Response snapshot

Updated the project constitution file with the specified principles and guidelines for the Spec-Driven AI Book with Embedded RAG Chatbot project. The constitution now includes six core principles covering spec-driven development, grounded content, developer clarity, modular architecture, ethical AI usage, and free-tier service compatibility. Additional sections cover constraints, development workflow, and governance.

## Outcome

- ✅ Impact: Created project constitution with 6 core principles and governance structure
- 🧪 Tests: No tests applicable for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Ready to proceed with feature specification and planning
- 🧠 Reflection: Constitution provides clear governance framework for the AI book project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin feature specification using /sp.specify