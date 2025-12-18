<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A
Added sections: Core Principles (6 total), Additional Constraints, Development Workflow
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ⚠ pending
Templates updated: 3/4
Runtime docs requiring updates: README.md ⚠ pending
Files requiring manual follow-up: 2power
Deferred placeholders: None
-->

# Spec-Driven AI Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven, Deterministic Development
Every feature and component must be defined through clear specifications before implementation; All development follows the Spec-Kit Plus methodology with deterministic, reproducible outcomes; No ad-hoc feature additions without proper specification and planning.

### II. No Hallucinations; Content Must Be Grounded
All AI-generated responses must be strictly grounded in source material; No creative extrapolation or assumption-making beyond provided content; RAG system must cite sources and limit answers to available book content only.

### III. Clarity for Developers (CS/AI Background)
Documentation and code must be accessible to developers with computer science and AI backgrounds; Clear, instructional, code-first approach in all materials; Comprehensive examples and well-documented APIs.

### IV. Reproducible, Modular Architecture
System components must be modular, independently deployable, and easily reproducible; Architecture should follow clean separation of concerns; Easy setup from fresh clone required.

### V. Ethical and Privacy-Aware AI Usage
All AI implementations must follow ethical guidelines and protect user privacy; Proper handling of data and user interactions; Transparent AI decision-making where applicable.

### VI. Free-Tier Service Compatibility
All infrastructure and services must operate within free-tier limitations of cloud providers; Architecture designed for cost-effectiveness and accessibility; Environment-based configuration for different deployment targets.

## Additional Constraints

Technology Stack: Spec-Kit Plus, Claude Code, Docusaurus for book; OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud (Free Tier) for RAG chatbot.

Deployment: GitHub Pages for book hosting with embedded RAG chatbot; End-to-end functionality from fresh clone required.

Security: Secure key handling with environment variables; No hardcoded secrets; Documented API setup and configuration steps.

Content Standards: Book content follows structured format (Concepts → Architecture → Implementation → Deployment); RAG retrieval strictly from book content; Selected-text queries answered exclusively from provided text.

## Development Workflow

Specification-Driven Process: All features begin with detailed specifications using Spec-Kit Plus templates; Implementation follows specifications precisely with regular validation against requirements.

Testing Requirements: All functionality must be tested with clear acceptance criteria; Both book content and RAG chatbot functionality must be validated; Integration tests for end-to-end workflows.

Review Process: Code reviews must verify compliance with constitutional principles; Architecture decisions documented in ADRs when significant; All changes must maintain free-tier compatibility.

Quality Gates: Book must compile and deploy successfully; RAG chatbot must answer content-based queries correctly; Project must run end-to-end from fresh clone.

## Governance

This constitution supersedes all other development practices and guidelines. All project decisions must align with these principles. Amendments require explicit documentation of changes, approval from project stakeholders, and a migration plan for existing code. All pull requests and code reviews must verify compliance with constitutional principles. Complexity must be justified with clear benefits. Use CLAUDE.md for runtime development guidance.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
