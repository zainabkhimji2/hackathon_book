"""
Content processing service for markdown files
"""
import os
import re
from typing import List, Dict
from pathlib import Path
import tiktoken

class ContentProcessor:
    def __init__(self):
        self.encoding = tiktoken.get_encoding("cl100k_base")
        self.chunk_size = int(os.getenv("CHUNK_SIZE", "600"))
        self.chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "75"))

    def count_tokens(self, text: str) -> int:
        """Count tokens in text"""
        return len(self.encoding.encode(text))

    def extract_metadata_from_path(self, file_path: str) -> Dict:
        """Extract metadata from file path"""
        path = Path(file_path)
        parts = path.parts

        # Extract week/chapter from path
        week_match = re.search(r'week-(\d+)', str(path))
        week = week_match.group(1) if week_match else "unknown"

        return {
            "file_path": str(file_path),
            "filename": path.name,
            "week": week,
            "directory": str(path.parent)
        }

    def split_by_headers(self, content: str, file_path: str) -> List[Dict]:
        """Split markdown content by headers (h2/h3)"""
        chunks = []

        # Extract title from first h1
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        title = title_match.group(1) if title_match else Path(file_path).stem

        # Split by h2 headers
        sections = re.split(r'\n##\s+', content)

        base_metadata = self.extract_metadata_from_path(file_path)
        base_metadata["title"] = title

        for i, section in enumerate(sections):
            if i == 0:  # First section might contain h1 and intro
                section_title = "Introduction"
                section_content = section
            else:
                # Extract section title
                lines = section.split('\n', 1)
                section_title = lines[0].strip()
                section_content = lines[1] if len(lines) > 1 else ""

            # Further split by h3 if section is too long
            subsections = re.split(r'\n###\s+', section_content)

            for j, subsection in enumerate(subsections):
                if j == 0:
                    subsection_title = section_title
                    subsection_content = subsection
                else:
                    lines = subsection.split('\n', 1)
                    subsection_title = f"{section_title} - {lines[0].strip()}"
                    subsection_content = lines[1] if len(lines) > 1 else ""

                # Clean content
                clean_content = subsection_content.strip()
                if not clean_content:
                    continue

                # Check if we need to further chunk based on token count
                token_count = self.count_tokens(clean_content)

                if token_count <= self.chunk_size:
                    chunk_metadata = base_metadata.copy()
                    chunk_metadata.update({
                        "section": subsection_title,
                        "page_url": f"/{base_metadata['week']}/{Path(file_path).stem}",
                        "chunk_index": len(chunks)
                    })

                    chunks.append({
                        "content": clean_content,
                        "metadata": chunk_metadata
                    })
                else:
                    # Split large sections with overlap
                    sub_chunks = self.chunk_with_overlap(clean_content)
                    for idx, sub_chunk in enumerate(sub_chunks):
                        chunk_metadata = base_metadata.copy()
                        chunk_metadata.update({
                            "section": f"{subsection_title} (part {idx + 1})",
                            "page_url": f"/{base_metadata['week']}/{Path(file_path).stem}",
                            "chunk_index": len(chunks)
                        })

                        chunks.append({
                            "content": sub_chunk,
                            "metadata": chunk_metadata
                        })

        return chunks

    def chunk_with_overlap(self, text: str) -> List[str]:
        """Split text into chunks with overlap"""
        tokens = self.encoding.encode(text)
        chunks = []

        start = 0
        while start < len(tokens):
            end = start + self.chunk_size
            chunk_tokens = tokens[start:end]
            chunk_text = self.encoding.decode(chunk_tokens)
            chunks.append(chunk_text)

            # Move start forward, accounting for overlap
            start = end - self.chunk_overlap

        return chunks

    def process_markdown_file(self, file_path: str) -> List[Dict]:
        """Process a single markdown file"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        return self.split_by_headers(content, file_path)

    def process_directory(self, directory: str) -> List[Dict]:
        """Process all markdown files in a directory"""
        all_chunks = []
        docs_path = Path(directory)

        for md_file in docs_path.rglob("*.md"):
            try:
                chunks = self.process_markdown_file(str(md_file))
                all_chunks.extend(chunks)
                print(f"Processed {md_file}: {len(chunks)} chunks")
            except Exception as e:
                print(f"Error processing {md_file}: {e}")

        return all_chunks

# Global instance
content_processor = ContentProcessor()
