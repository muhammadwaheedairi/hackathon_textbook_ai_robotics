const API_CONFIG = {
  baseUrl: 'https://muhammadwaheedairi-rag-chatbot-textbook.hf.space', // Hugging Face Space URL
  timeout: 60000, // 60 seconds timeout to accommodate HF Spaces cold starts
  maxRetries: 3,
  headers: {
    'Content-Type': 'application/json',
  },
};

// API endpoints
export const API_ENDPOINTS = {
  ask: '/ask',
  health: '/health',
} as const;

// Export the configuration
export default API_CONFIG;
