// API service for RAG Agent integration

import { QueryRequest, QueryResponse } from '../../components/RagChatbot/RagChatbot.types';
import { API_CONFIG, API_ENDPOINTS } from './config';

/**
 * Calls the backend RAG agent API to answer a question
 * @param request - The query request containing the question
 * @returns Promise resolving to the query response
 */
export const askRagAgent = async (request: QueryRequest): Promise<QueryResponse> => {
  try {
    const response = await fetch(`${API_CONFIG.baseUrl}${API_ENDPOINTS.ask}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      // Handle different status codes
      if (response.status === 400) {
        return {
          answer: '',
          sources: [],
          matched_chunks: [],
          error: 'Bad request - query validation failed',
          status: 'error',
        };
      } else if (response.status >= 500) {
        return {
          answer: '',
          sources: [],
          matched_chunks: [],
          error: 'Internal server error occurred',
          status: 'error',
        };
      } else {
        return {
          answer: '',
          sources: [],
          matched_chunks: [],
          error: `Request failed with status ${response.status}`,
          status: 'error',
        };
      }
    }

    const data: QueryResponse = await response.json();
    return data;
  } catch (error) {
    // Handle network errors
    return {
      answer: '',
      sources: [],
      matched_chunks: [],
      error: error instanceof Error ? error.message : 'Network error occurred',
      status: 'error',
    };
  }
};

/**
 * Health check for the RAG API
 * @returns Promise resolving to health status
 */
export const checkRagApiHealth = async (): Promise<boolean> => {
  try {
    const response = await fetch(`${API_CONFIG.baseUrl}${API_ENDPOINTS.health}`);
    return response.ok;
  } catch (error) {
    return false;
  }
};