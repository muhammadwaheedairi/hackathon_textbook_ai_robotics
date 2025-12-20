// Type definitions for RAG Chatbot component

export interface QueryRequest {
  query: string;
}

export interface MatchedChunk {
  content: string;
  url: string;
  position: number;
  similarity_score: number;
}

export interface QueryResponse {
  answer: string;
  sources: string[];
  matched_chunks: MatchedChunk[];
  error?: string;
  status: 'success' | 'error' | 'empty';
  query_time_ms?: number;
  confidence?: string;
}

export interface RagChatbotState {
  query: string;
  isLoading: boolean;
  answer: QueryResponse | null;
  error: string | null;
  showResults: boolean;
}

export interface RagChatbotProps {
  initialQuery?: string;
  onSubmit?: (query: string) => void;
  onResponse?: (response: QueryResponse) => void;
}