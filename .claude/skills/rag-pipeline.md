---
description: Implement Retrieval-Augmented Generation systems - content ingestion, chunking, embedding, vector search, and LLM generation.
---

Implement Retrieval-Augmented Generation systems: content ingestion, chunking, embedding, vector search, and LLM generation.

WHAT IT DOES:
Builds complete RAG pipelines from markdown to chatbot responses. Handles chunking strategies, embedding generation, vector storage, retrieval, and prompt construction.

RAG COMPONENTS:
1. Content Ingestion: Read files, parse markdown
2. Chunking: Split by headers, ~500 tokens, overlap
3. Embedding: OpenAI text-embedding-3-small
4. Storage: Qdrant with metadata
5. Retrieval: Vector similarity search (top-k)
6. Augmentation: Build context-rich prompts
7. Generation: LLM with retrieved context

CHUNKING STRATEGY:
- Split on headers (h2, h3)
- 300-700 token chunks
- 50-100 token overlap
- Preserve code blocks intact
- Include metadata (page, section)

USE FOR: Building the textbook chatbot, implementing Q&A systems, creating semantic search.
