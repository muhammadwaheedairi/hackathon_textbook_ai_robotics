// RagChatbot.tsx

// Message type
export type Message = {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
};

// QueryRequest type
export type QueryRequest = {
  query: string;
};

// API response type (sources kept, matched_chunks removed)
export interface QueryResponse {
  answer: string;
  sources?: string[];
  error?: string;
  status: 'success' | 'error' | 'empty';
}

export interface ChatWidgetProps {
  initialQuery?: string;
  onSubmit?: (query: string) => void;
  onResponse?: (response: QueryResponse) => void;
}

export interface ChatState {
  query: string;
  isLoading: boolean;
  answer: QueryResponse | null;
  error: string | null;
  showResults: boolean;
}
