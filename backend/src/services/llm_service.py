"""
OpenAI LLM service for answer generation
"""
import os
from typing import List, Dict
from openai import OpenAI

class LLMService:
    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = os.getenv("OPENAI_CHAT_MODEL", "gpt-4-turbo-preview")

    def generate_answer(
        self,
        question: str,
        context_chunks: List[Dict],
        conversation_history: List[Dict] = None,
        selected_text: str = None
    ) -> str:
        """Generate answer using RAG approach"""

        # Build context from retrieved chunks
        context = "\n\n".join([
            f"[Source: {chunk['metadata'].get('title', 'Unknown')} - {chunk['metadata'].get('section', '')}]\n{chunk['content']}"
            for chunk in context_chunks
        ])

        # Build system message
        system_message = """You are a helpful teaching assistant for a Physical AI and robotics textbook.
Your role is to help students understand ROS 2, robotics concepts, and Physical AI topics.

IMPORTANT INSTRUCTIONS:
1. Answer questions ONLY using information from the provided textbook sources
2. If the answer is not in the sources, clearly state "I don't have information about this in the textbook"
3. Include citations by referencing the source sections (e.g., "According to the Week 3: First Python Node section...")
4. Format citations as markdown links when possible: [Section Name](URL)
5. Be clear, educational, and encourage learning
6. If code examples are in the sources, include them in your answer
7. Keep answers concise (300-500 words) but complete

SOURCES:
{context}
""".format(context=context)

        # Build messages
        messages = [{"role": "system", "content": system_message}]

        # Add conversation history if available
        if conversation_history:
            for msg in conversation_history[-6:]:  # Last 3 exchanges
                messages.append(msg)

        # Add current question
        user_message = question
        if selected_text:
            user_message = f"The user highlighted this text: \"{selected_text}\"\n\nTheir question: {question}"

        messages.append({"role": "user", "content": user_message})

        # Generate response
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.7,
            max_tokens=800
        )

        return response.choices[0].message.content

# Global instance
llm_service = LLMService()
