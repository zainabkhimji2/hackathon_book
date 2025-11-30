"""
Content ingestion script
Processes markdown files and stores embeddings in Qdrant
"""
import os
import sys
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from dotenv import load_dotenv
from qdrant_client.models import PointStruct

# Load .env from backend directory
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

from src.services.content_processor import content_processor
from src.services.embedding_service import embedding_service
from src.services.qdrant_service import qdrant_service

def ingest_content():
    """Main ingestion function"""
    docs_dir = os.getenv("DOCS_DIR", "../physical-ai-textbook/docs")

    print(f"Processing markdown files from: {docs_dir}")

    # Check if directory exists
    if not os.path.exists(docs_dir):
        print(f"Error: Directory {docs_dir} not found")
        print("Please update DOCS_DIR in .env file")
        return

    # Process all markdown files
    print("Step 1: Processing markdown files...")
    chunks = content_processor.process_directory(docs_dir)
    print(f"Total chunks created: {len(chunks)}")

    if not chunks:
        print("No chunks created. Check if markdown files exist in the directory.")
        return

    # Generate embeddings
    print("Step 2: Generating embeddings...")
    batch_size = 100
    points = []

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [chunk["content"] for chunk in batch]

        # Generate embeddings for batch
        embeddings = embedding_service.generate_embeddings(texts)

        # Create points for Qdrant
        for j, (chunk, embedding) in enumerate(zip(batch, embeddings)):
            point = PointStruct(
                id=i + j,
                vector=embedding,
                payload={
                    "content": chunk["content"],
                    **chunk["metadata"]
                }
            )
            points.append(point)

        print(f"Processed {min(i + batch_size, len(chunks))}/{len(chunks)} chunks")

    # Connect to Qdrant and create collection
    print("Step 3: Connecting to Qdrant...")
    qdrant_service.connect()
    qdrant_service.create_collection(vector_size=1536)

    # Upload points to Qdrant
    print("Step 4: Uploading to Qdrant...")
    upload_batch_size = 50  # Smaller batches to avoid timeout
    for i in range(0, len(points), upload_batch_size):
        batch = points[i:i + upload_batch_size]
        try:
            qdrant_service.upsert_points(batch)
            print(f"Uploaded {min(i + upload_batch_size, len(points))}/{len(points)} points")
        except Exception as e:
            print(f"Error uploading batch {i}: {e}")
            print("Retrying with smaller batch...")
            # Retry with even smaller batches
            for j in range(0, len(batch), 10):
                mini_batch = batch[j:j + 10]
                qdrant_service.upsert_points(mini_batch)
            print(f"Uploaded {min(i + upload_batch_size, len(points))}/{len(points)} points (retry succeeded)")

    print("Content ingestion complete!")
    print(f"Total points indexed: {len(points)}")

if __name__ == "__main__":
    ingest_content()
