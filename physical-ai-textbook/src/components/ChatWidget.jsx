import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatWidget.module.css';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const API_URL = 'http://localhost:8000/api';

  useEffect(() => {
    // Generate session ID
    if (!sessionId) {
      setSessionId('session-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9));
    }

    // Text selection handler
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      if (text.length > 0) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => document.removeEventListener('mouseup', handleTextSelection);
  }, [sessionId]);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim() || loading) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/ask`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: input,
          selected_text: selectedText || null,
          current_page: window.location.pathname,
          session_id: sessionId
        })
      });

      const data = await response.json();

      if (data.error) {
        throw new Error(data.error);
      }

      const botMessage = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations || []
      };

      setMessages(prev => [...prev, botMessage]);
      setSelectedText(''); // Clear selected text after use
    } catch (error) {
      console.error('Error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        error: true
      }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <h3>AI Teaching Assistant</h3>
            <button onClick={() => setIsOpen(false)} className={styles.closeBtn}>✕</button>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I'm your AI teaching assistant.</p>
                <p>Ask me anything about the textbook content!</p>
                {selectedText && (
                  <p className={styles.selectedTextHint}>
                    💡 You've selected text. Ask about it!
                  </p>
                )}
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
                <div className={styles.messageContent}>
                  {msg.content}
                  {msg.citations && msg.citations.length > 0 && (
                    <div className={styles.citations}>
                      <p><strong>Sources:</strong></p>
                      {msg.citations.map((citation, i) => (
                        <a
                          key={i}
                          href={citation.url}
                          className={styles.citation}
                          target="_blank"
                          rel="noopener noreferrer"
                        >
                          📖 {citation.section}
                        </a>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {loading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.loader}>Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={sendMessage} className={styles.chatInput}>
            {selectedText && (
              <div className={styles.selectedTextBadge}>
                Selected: "{selectedText.substring(0, 50)}..."
                <button type="button" onClick={() => setSelectedText('')}>✕</button>
              </div>
            )}
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              disabled={loading}
            />
            <button type="submit" disabled={loading || !input.trim()}>
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
}
