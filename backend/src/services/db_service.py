"""
PostgreSQL database service for conversation history
"""
import os
import psycopg2
from psycopg2.extras import RealDictCursor
from typing import List, Dict, Optional
import json
from datetime import datetime

class DatabaseService:
    def __init__(self):
        self.database_url = os.getenv("DATABASE_URL")
        self.conn = None

    def connect(self):
        """Connect to PostgreSQL database"""
        if not self.conn or self.conn.closed:
            self.conn = psycopg2.connect(self.database_url)
        return self.conn

    def create_tables(self):
        """Create necessary tables if they don't exist"""
        with self.connect().cursor() as cur:
            cur.execute("""
                CREATE TABLE IF NOT EXISTS conversations (
                    id SERIAL PRIMARY KEY,
                    session_id VARCHAR(255) NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );

                CREATE INDEX IF NOT EXISTS idx_session_id ON conversations(session_id);

                CREATE TABLE IF NOT EXISTS messages (
                    id SERIAL PRIMARY KEY,
                    conversation_id INTEGER REFERENCES conversations(id) ON DELETE CASCADE,
                    role VARCHAR(50) NOT NULL,
                    content TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );

                CREATE INDEX IF NOT EXISTS idx_conversation_id ON messages(conversation_id);
            """)
            self.conn.commit()

    def get_or_create_conversation(self, session_id: str) -> int:
        """Get existing conversation or create new one"""
        with self.connect().cursor() as cur:
            # Try to find existing conversation
            cur.execute(
                "SELECT id FROM conversations WHERE session_id = %s ORDER BY created_at DESC LIMIT 1",
                (session_id,)
            )
            result = cur.fetchone()

            if result:
                return result[0]

            # Create new conversation
            cur.execute(
                "INSERT INTO conversations (session_id) VALUES (%s) RETURNING id",
                (session_id,)
            )
            self.conn.commit()
            return cur.fetchone()[0]

    def save_message(self, conversation_id: int, role: str, content: str):
        """Save a message to the conversation"""
        with self.connect().cursor() as cur:
            cur.execute(
                "INSERT INTO messages (conversation_id, role, content) VALUES (%s, %s, %s)",
                (conversation_id, role, content)
            )
            self.conn.commit()

    def get_conversation_history(self, session_id: str, limit: int = 10) -> List[Dict]:
        """Get conversation history for a session"""
        with self.connect().cursor(cursor_factory=RealDictCursor) as cur:
            cur.execute("""
                SELECT m.role, m.content, m.created_at
                FROM messages m
                JOIN conversations c ON m.conversation_id = c.id
                WHERE c.session_id = %s
                ORDER BY m.created_at ASC
                LIMIT %s
            """, (session_id, limit))

            messages = cur.fetchall()
            return [
                {"role": msg["role"], "content": msg["content"]}
                for msg in messages
            ]

    def health_check(self) -> bool:
        """Check if database is accessible"""
        try:
            with self.connect().cursor() as cur:
                cur.execute("SELECT 1")
                return True
        except Exception as e:
            print(f"Database health check failed: {e}")
            return False

    def close(self):
        """Close database connection"""
        if self.conn and not self.conn.closed:
            self.conn.close()

# Global instance
db_service = DatabaseService()
