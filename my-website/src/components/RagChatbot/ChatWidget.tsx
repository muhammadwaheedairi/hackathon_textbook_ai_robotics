import React, { useState, useEffect, useCallback, useRef, KeyboardEvent } from 'react';
import { askRagAgent } from '../../services/api/ragService';
import { debounce } from '../../utils/debounce';

// Define message types
type Message = {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
};

// Define response types
interface MatchedChunk {
  content: string;
  url: string;
  position: number;
  similarity_score: number;
}

interface QueryResponse {
  answer: string;
  sources: string[];
  matched_chunks: MatchedChunk[];
  error?: string;
  status: 'success' | 'error' | 'empty';
  query_time_ms?: number;
  confidence?: string;
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

  // Cleanup function to set mounted state to false on unmount
  useEffect(() => {
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages, state.isLoading]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Toggle chat widget open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Close chat widget
  const closeChat = () => {
    setIsOpen(false);
  };

  // Handle input change
  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setState(prev => ({
      ...prev,
      query: e.target.value,
      error: null,
    }));
  };

  // Handle form submission
  const handleSubmitImmediate = useCallback(async (e?: React.FormEvent) => {
    if (e) {
      e.preventDefault();
    }

    const currentQuery = state.query.trim(); // Capture current query to avoid stale closure

    if (!currentQuery) {
      setState(prev => ({
        ...prev,
        error: 'Please enter a question',
      }));
      return;
    }

    // Add user message to chat
    const userMessage: Message = {
      id: Date.now().toString(),
      text: currentQuery,
      sender: 'user',
      timestamp: new Date(),
    };
    setMessages(prev => [...prev, userMessage]);

    try {
      setState(prev => ({
        ...prev,
        isLoading: true,
        error: null,
      }));

      // Call the API service
      const response = await askRagAgent({ query: currentQuery });

      // Check if component is still mounted before updating state
      if (isMountedRef.current) {
        // Handle response based on status
        if (response.status === 'error') {
          // Handle API error responses
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: response.error || 'An error occurred while processing your request.',
          }));

          // Add error message to chat
          const errorMessage: Message = {
            id: Date.now().toString(),
            text: response.error || 'An error occurred while processing your request.',
            sender: 'bot',
            timestamp: new Date(),
          };
          setMessages(prev => [...prev, errorMessage]);
        } else if (response.status === 'empty' ||
                  (!response.answer?.trim() &&
                   response.sources.length === 0 &&
                   response.matched_chunks.length === 0)) {
          // Handle empty response case
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: 'No answer found for your question. Please try rephrasing.',
          }));

          // Add empty response message to chat
          const emptyMessage: Message = {
            id: Date.now().toString(),
            text: 'No answer found for your question. Please try rephrasing.',
            sender: 'bot',
            timestamp: new Date(),
          };
          setMessages(prev => [...prev, emptyMessage]);
        } else {
          // Handle successful response - only show results if there's actual content
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: null,
          }));

          // Add bot response to chat
          const botMessage: Message = {
            id: Date.now().toString(),
            text: response.answer || 'I found some information for you.',
            sender: 'bot',
            timestamp: new Date(),
          };
          setMessages(prev => [...prev, botMessage]);
        }

        // Call the onSubmit callback if provided
        onSubmit?.(currentQuery);
        onResponse?.(response);
      }
    } catch (error) {
      // Check if component is still mounted before updating state
      if (!isMountedRef.current) return;

      // Handle network and other errors
      let errorMessage = 'An error occurred while processing your request.';
      if (error instanceof Error) {
        if (error.message.includes('Network Error') || error.message.includes('Failed to fetch')) {
          errorMessage = 'Network error: Unable to connect to the server. This may be due to the backend being temporarily inactive. Please try again in a moment.';
        } else if (error.message.includes('timeout') || error.message.includes('timeout')) {
          errorMessage = 'Request timeout: The server is taking too long to respond. This may be due to the backend being temporarily inactive. Please try again.';
        } else {
          errorMessage = error.message;
        }
      }

      setState(prev => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
        showResults: false, // Don't show results if there's an error
      }));

      // Add error message to chat
      const errorMessageObj: Message = {
        id: Date.now().toString(),
        text: errorMessage,
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessageObj]);
    }
  }, [state.query, onSubmit, onResponse, isMountedRef]);

  // Handle form submission - call immediate version directly to avoid debounce issues
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    handleSubmitImmediate(e);
  };

  // Handle key press for sending message
  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmitImmediate();
    }
  };

  // Implement retry functionality
  const handleRetry = useCallback(async () => {
    const currentQuery = state.query.trim(); // Capture current query to avoid stale closure

    if (!currentQuery) return;

    try {
      setState(prev => ({
        ...prev,
        isLoading: true,
        error: null,
      }));

      // Call the API service again
      const response = await askRagAgent({ query: currentQuery });

      // Check if component is still mounted before updating state
      if (isMountedRef.current) {
        // Handle response based on status
        if (response.status === 'error') {
          // Handle API error responses
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: response.error || 'An error occurred while processing your request.',
          }));

          // Add error message to chat
          const errorMessage: Message = {
            id: Date.now().toString(),
            text: response.error || 'An error occurred while processing your request.',
            sender: 'bot',
            timestamp: new Date(),
          };
          setMessages(prev => [...prev, errorMessage]);
        } else if (response.status === 'empty' ||
                  (!response.answer?.trim() &&
                   response.sources.length === 0 &&
                   response.matched_chunks.length === 0)) {
          // Handle empty response case
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: 'No answer found for your question. Please rephrasing.',
          }));

          // Add empty response message to chat
          const emptyMessage: Message = {
            id: Date.now().toString(),
            text: 'No answer found for your question. Please try rephrasing.',
            sender: 'bot',
            timestamp: new Date(),
          };
          setMessages(prev => [...prev, emptyMessage]);
        } else {
          // Handle successful response - only show results if there's actual content
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: null,
          }));

          // Add bot response to chat
          const botMessage: Message = {
            id: Date.now().toString(),
            text: response.answer || 'I found some information for you.',
            sender: 'bot',
            timestamp: new Date(),
          };
          setMessages(prev => [...prev, botMessage]);
        }

        // Call the onSubmit callback if provided
        onSubmit?.(currentQuery);
        onResponse?.(response);
      }
    } catch (error) {
      // Check if component is still mounted before updating state
      if (!isMountedRef.current) return;

      // Handle network and other errors
      let errorMessage = 'An error occurred while processing your request.';
      if (error instanceof Error) {
        if (error.message.includes('Network Error') || error.message.includes('Failed to fetch')) {
          errorMessage = 'Network error: Unable to connect to the server. This may be due to the backend being temporarily inactive. Please try again in a moment.';
        } else if (error.message.includes('timeout') || error.message.includes('timeout')) {
          errorMessage = 'Request timeout: The server is taking too long to respond. This may be due to the backend being temporarily inactive. Please try again.';
        } else {
          errorMessage = error.message;
        }
      }

      setState(prev => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
        showResults: false, // Don't show results if there's an error
      }));

      // Add error message to chat
      const errorMessageObj: Message = {
        id: Date.now().toString(),
        text: errorMessage,
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessageObj]);
    }
  }, [state.query, onSubmit, onResponse, isMountedRef]);

  return (
    <>
      {/* Floating chat button */}
      <button
        className="floating-chat-button"
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
        aria-expanded={isOpen}
      >
        💬
      </button>

      {/* Chat container */}
      <div className={`chat-container ${isOpen ? 'visible' : ''}`}>
        {/* Chat header */}
        <div className="chat-header">
          <span>AI Assistant</span>
          <button
            className="close-button"
            onClick={closeChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        {/* Messages area */}
        <div className="messages-area">
          {messages.map((message) => (
            <div
              key={message.id}
              className={`message-bubble ${
                message.sender === 'user' ? 'user-message' : 'bot-message'
              }`}
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
              {state.error.includes('Network error') && (
                <button
                  type="button"
                  className="retry-button"
                  onClick={handleRetry}
                  aria-label="Retry the request"
                >
                  Retry
                </button>
              )}
            </div>
          )}

          {messages.length === 0 && !state.isLoading && (
            <div className="empty-state bot-message">
              How can I help you today?
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Input area */}
        <form onSubmit={handleSubmit} className="input-area">
          <textarea
            value={state.query}
            onChange={handleInputChange}
            onKeyPress={handleKeyPress}
            placeholder="Type your message..."
            className="input-field"
            rows={1}
            disabled={state.isLoading}
            aria-label="Type your message"
          />
          <button
            type="submit"
            className="send-button"
            disabled={state.isLoading || !state.query.trim()}
            aria-label={state.isLoading ? "Sending message, please wait" : "Send message"}
          >
            {state.isLoading ? '...' : 'Send'}
          </button>
        </form>
      </div>
    </>
  );
};

export default ChatWidget;