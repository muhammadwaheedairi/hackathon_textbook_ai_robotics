import React, { useState, useEffect, useCallback, useRef } from 'react';
import { askRagAgent } from '../../services/api/ragService';
import { RagChatbotProps, RagChatbotState, QueryResponse } from './RagChatbot.types';
import styles from './RagChatbot.module.css';
import { debounce } from '../../utils/debounce';

const RagChatbot: React.FC<RagChatbotProps> = ({
  initialQuery = '',
  onSubmit,
  onResponse
}) => {
  const [state, setState] = useState<RagChatbotState>({
    query: initialQuery,
    isLoading: false,
    answer: null,
    error: null,
    showResults: false,
  });

  // Create a ref to hold the mounted state
  const isMountedRef = useRef(true);

  // Cleanup function to set mounted state to false on unmount
  useEffect(() => {
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Handle input change
  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setState(prev => ({
      ...prev,
      query: e.target.value,
      error: null,
    }));
  };

  // Handle form submission (immediate version for debouncing)
  const handleSubmitImmediate = useCallback(async (e: React.FormEvent) => {
    e.preventDefault();

    const currentQuery = state.query; // Capture current query to avoid stale closure

    if (!currentQuery.trim()) {
      setState(prev => ({
        ...prev,
        error: 'Please enter a question',
      }));
      return;
    }

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
        } else {
          // Handle successful response - only show results if there's actual content
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: null,
          }));
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
          errorMessage = 'Network error: Unable to connect to the server. Please check your connection and try again.';
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
    }
  }, [state.query, onSubmit, onResponse, isMountedRef]);

  // Create debounced version of handleSubmitImmediate to prevent excessive API calls
  const debouncedSubmit = useCallback(
    debounce(handleSubmitImmediate, 300),
    [handleSubmitImmediate]
  );

  // Handle form submission - call immediate version directly to avoid debounce issues
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    handleSubmitImmediate(e);
  };

  // Implement retry functionality
  const handleRetry = useCallback(async () => {
    const currentQuery = state.query; // Capture current query to avoid stale closure

    if (!currentQuery.trim()) return;

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
        } else {
          // Handle successful response - only show results if there's actual content
          setState(prev => ({
            ...prev,
            isLoading: false,
            answer: response,
            showResults: true,
            error: null,
          }));
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
          errorMessage = 'Network error: Unable to connect to the server. Please check your connection and try again.';
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
    }
  }, [state.query, onSubmit, onResponse, isMountedRef]);

  return (
    <div className={styles.container} role="region" aria-label="RAG Question and Answer Interface">
      <form onSubmit={handleSubmit} className={styles.form} role="form" aria-label="Question submission form">
        <div className={styles.inputContainer}>
          <label htmlFor="rag-question-input" className={styles.inputLabel}>
            Ask a question about the textbook content:
          </label>
          <textarea
            id="rag-question-input"
            value={state.query}
            onChange={handleInputChange}
            placeholder="Ask a question about the textbook content..."
            className={styles.textarea}
            rows={3}
            disabled={state.isLoading}
            aria-describedby={state.error ? "rag-error-message" : undefined}
            aria-invalid={!!state.error}
            aria-required="true"
          />
          <button
            type="submit"
            className={styles.submitButton}
            disabled={state.isLoading || !state.query.trim()}
            aria-label={state.isLoading ? "Sending question, please wait" : "Submit question"}
          >
            {state.isLoading ? 'Sending...' : 'Ask'}
          </button>
        </div>

        {state.error && (
          <div
            id="rag-error-message"
            className={styles.error}
            role="alert"
            aria-live="polite"
          >
            <div className={styles.errorContent}>
              <span>{state.error}</span>
              {state.error.includes('Network error') && (
                <button
                  type="button"
                  className={styles.retryButton}
                  onClick={handleRetry}
                  aria-label="Retry the request"
                >
                  Retry
                </button>
              )}
            </div>
          </div>
        )}

        {/* Input validation feedback */}
        {state.query.length > 0 && state.query.length < 10 && (
          <div
            className={styles.validationFeedback}
            role="status"
            aria-live="polite"
          >
            Question should be more descriptive for better results.
          </div>
        )}
      </form>

      {state.isLoading && (
        <div
          className={styles.loading}
          role="status"
          aria-live="polite"
        >
          <div className={styles.spinner} aria-hidden="true"></div>
          <p>Processing your question...</p>
          {/* Loading skeleton UI for better perceived performance */}
          <div className={styles.skeletonContainer}>
            <div className={styles.skeletonHeader}></div>
            <div className={styles.skeletonText}></div>
            <div className={styles.skeletonText}></div>
            <div className={styles.skeletonText}></div>
            <div className={styles.skeletonSources}>
              <div className={styles.skeletonSource}></div>
              <div className={styles.skeletonSource}></div>
            </div>
          </div>
        </div>
      )}

      {state.showResults && state.answer && !state.isLoading && (
        <>
          {/* Display empty state message when no answer is found */}
          {state.answer.status === 'empty' || (!state.answer.answer && state.answer.sources.length === 0 && state.answer.matched_chunks.length === 0) ? (
            <div
              className={styles.noAnswer}
              role="status"
              aria-live="polite"
            >
              <h3>No Answer Found</h3>
              <p>We couldn't find an answer for your question in the textbook content.</p>
              <p>Please try rephrasing your question or asking about a different topic.</p>
            </div>
          ) : (
            <div className={styles.results} role="region" aria-label="Question response results">
              <div className={styles.answer} role="region" aria-label="Generated answer">
                <h3 className={styles.answerHeader}>Answer</h3>
                <p className={styles.answerText}>{state.answer.answer}</p>
              </div>

              {state.answer.sources.length > 0 && (
                <div className={styles.sources} role="region" aria-label="Supporting sources">
                  <h3 className={styles.sectionHeader}>Supporting Sources</h3>
                  <ul className={styles.sourcesList}>
                    {state.answer.sources.map((source, index) => (
                      <li key={index} className={styles.sourceItem}>
                        <a
                          href={source}
                          target="_blank"
                          rel="noopener noreferrer"
                          className={styles.sourceLink}
                        >
                          {source}
                        </a>
                      </li>
                    ))}
                  </ul>
                </div>
              )}

              {state.answer.matched_chunks.length > 0 && (
                <div className={styles.chunks} role="region" aria-label="Relevant text chunks">
                  <h3 className={styles.sectionHeader}>Relevant Text Chunks</h3>
                  {state.answer.matched_chunks.map((chunk, index) => (
                    <div key={index} className={styles.chunk}>
                      <p className={styles.chunkContent}>{chunk.content}</p>
                      <div className={styles.chunkMeta}>
                        <a
                          href={chunk.url}
                          target="_blank"
                          rel="noopener noreferrer"
                          className={styles.chunkSource}
                        >
                          Source
                        </a>
                        <span className={styles.similarityScore} aria-label={`Similarity score: ${(chunk.similarity_score * 100).toFixed(1)}%`}>
                          Similarity: {(chunk.similarity_score * 100).toFixed(1)}%
                        </span>
                      </div>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}
        </>
      )}
    </div>
  );
};

export default RagChatbot;