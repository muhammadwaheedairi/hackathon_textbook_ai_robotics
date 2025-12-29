import React, { useState, useEffect, useCallback, useRef, KeyboardEvent } from 'react';
import { askRagAgent } from '../../services/api/ragService';

// Message type
type Message = {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
};

// API response type (sources kept, matched_chunks removed)
interface QueryResponse {
  answer: string;
  sources?: string[];
  error?: string;
  status: 'success' | 'error' | 'empty';
}

interface ChatWidgetProps {
  initialQuery?: string;
  onSubmit?: (query: string) => void;
  onResponse?: (response: QueryResponse) => void;
}

interface ChatState {
  query: string;
  isLoading: boolean;
  answer: QueryResponse | null;
  error: string | null;
  showResults: boolean;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({
  initialQuery = '',
  onSubmit,
  onResponse
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [state, setState] = useState<ChatState>({
    query: initialQuery,
    isLoading: false,
    answer: null,
    error: null,
    showResults: false,
  });

  const [messages, setMessages] = useState<Message[]>([]);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const isMountedRef = useRef(true);

  useEffect(() => {
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, state.isLoading]);

  const toggleChat = () => setIsOpen(!isOpen);
  const closeChat = () => setIsOpen(false);

  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setState(prev => ({ ...prev, query: e.target.value, error: null }));
  };

  const handleSubmitImmediate = useCallback(async (e?: React.FormEvent) => {
    if (e) e.preventDefault();

    const currentQuery = state.query.trim();
    if (!currentQuery) {
      setState(prev => ({ ...prev, error: 'Please enter a question' }));
      return;
    }

    const userMessage: Message = {
      id: Date.now().toString(),
      text: currentQuery,
      sender: 'user',
      timestamp: new Date(),
    };
    setMessages(prev => [...prev, userMessage]);

    try {
      setState(prev => ({ ...prev, isLoading: true, error: null }));

      const response = await askRagAgent({ query: currentQuery });

      if (!isMountedRef.current) return;

      if (response.status === 'success' && response.answer.trim()) {
        setState(prev => ({
          ...prev,
          isLoading: false,
          answer: response,
          showResults: true,
          error: null,
        }));

        const botMessage: Message = {
          id: Date.now().toString(),
          text: response.answer,
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botMessage]);
      } else if (response.status === 'empty' || !response.answer.trim()) {
        setState(prev => ({
          ...prev,
          isLoading: false,
          answer: response,
          showResults: true,
          error: 'No answer found for your question. Please try rephrasing.',
        }));

        const emptyMessage: Message = {
          id: Date.now().toString(),
          text: 'No answer found for your question. Please try rephrasing.',
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, emptyMessage]);
      } else {
        setState(prev => ({
          ...prev,
          isLoading: false,
          answer: response,
          showResults: true,
          error: response.error || 'An error occurred.',
        }));

        const errorMessage: Message = {
          id: Date.now().toString(),
          text: response.error || 'An error occurred.',
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }

      onSubmit?.(currentQuery);
      onResponse?.(response);

    } catch (error) {
      if (!isMountedRef.current) return;

      let errorMessage = 'An error occurred while processing your request.';
      if (error instanceof Error) errorMessage = error.message;

      setState(prev => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
        showResults: false,
      }));

      const errorObj: Message = {
        id: Date.now().toString(),
        text: errorMessage,
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorObj]);
    }
  }, [state.query, onSubmit, onResponse]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    handleSubmitImmediate(e);
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmitImmediate();
    }
  };

  const handleRetry = useCallback(() => {
    handleSubmitImmediate();
  }, [handleSubmitImmediate]);

  return (
    <>
      <button
        className="floating-chat-button"
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
        aria-expanded={isOpen}
      >
        💬
      </button>

      <div className={`chat-container ${isOpen ? 'visible' : ''}`}>
        <div className="chat-header">
          <span>AI Assistant</span>
          <button className="close-button" onClick={closeChat}>×</button>
        </div>

        <div className="messages-area">
          {messages.map((message) => (
            <div
              key={message.id}
              className={`message-bubble ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
            >
              {message.text}
            </div>
          ))}

          {state.isLoading && (
            <div className="message-bubble typing-indicator bot-message">
              <div className="typing-dots">
                <div className="typing-dot"></div>
                <div className="typing-dot"></div>
                <div className="typing-dot"></div>
              </div>
            </div>
          )}

          {state.error && !state.isLoading && (
            <div className="message-bubble error-message bot-message">
              {state.error}
              <button className="retry-button" onClick={handleRetry}>Retry</button>
            </div>
          )}

          {messages.length === 0 && !state.isLoading && (
            <div className="empty-state bot-message">
              How can I help you today?
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className="input-area">
          <textarea
            value={state.query}
            onChange={handleInputChange}
            onKeyPress={handleKeyPress}
            placeholder="Type your message..."
            className="input-field"
            rows={1}
            disabled={state.isLoading}
          />
          <button
            type="submit"
            className="send-button"
            disabled={state.isLoading || !state.query.trim()}
          >
            {state.isLoading ? '...' : 'Send'}
          </button>
        </form>
      </div>
    </>
  );
};

export default ChatWidget;
