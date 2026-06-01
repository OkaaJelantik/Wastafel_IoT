# IoT Agentic Framework: Automated Pipeline

## 1. Roles & Responsibilities
- **Gemini CLI (Orchestrator):** High-level planning, context management, prompt engineering for other agents.
- **Perplexity (Researcher):** Deep-dive research into ESP-IDF components, hardware pins, and best practices.
- **Claude Opus (Architect):** One-time initialization of the core codebase based on Gemini's specs.
- **Subsequent Agents (Implementers):** Feature development and bug fixes.

## 2. Context Architecture
- `docs/agents/MASTER_PLAN.md`: The living document of project goals and status.
- `docs/agents/RESEARCH_LOG.md`: Compiled findings from Perplexity.
- `docs/agents/INSTRUCTION_PACKS/`: Versioned prompts for external agents.

## 3. Workflow Steps
1. **Research (Gemini + Perplexity):** Gemini generates a Research Brief. User feeds to Perplexity. Result goes into `RESEARCH_LOG.md`.
2. **Spec (Gemini):** Gemini synthesizes `MASTER_PLAN.md` and `RESEARCH_LOG.md` into a technical spec.
3. **Initialization (Gemini -> Claude Opus):** Gemini generates a "Genesis Prompt" (XML-wrapped) for Claude Opus.
4. **Execution (Gemini -> Code Agent):** Gemini generates "Task Packs" for the current implementation agent.

## 4. Token Optimization Strategy
- **Compression:** Gemini summarizes research into bullet points rather than raw text.
- **Directives:** Instruction packs use strict "System-like" directives to reduce conversational filler in external agents.
- **Context Pruning:** Old tasks are archived to keep the active context lean.
