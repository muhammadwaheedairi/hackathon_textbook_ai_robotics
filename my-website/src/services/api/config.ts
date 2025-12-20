// API Configuration for RAG Agent integration

// Helper function to safely get environment variables in browser
const getEnvVar = (name: string, defaultValue: string): string => {
  // For Docusaurus, we access custom fields through window.__APP_DATA__
  if (typeof window !== 'undefined' && (window as any).__APP_DATA__) {
    const appData = (window as any).__APP_DATA__;
    const siteConfig = appData?.siteConfig || {};
    const customFields = siteConfig.customFields || {};

    // Map our custom field names to the expected env var names
    if (name === 'REACT_APP_RAG_API_URL') {
      return customFields.RAG_API_URL || defaultValue;
    } else if (name === 'NODE_ENV') {
      return customFields.NODE_ENV || defaultValue;
    }
  }

  // Fallback to default value
  return defaultValue;
};

// Default API configuration
const DEFAULT_API_CONFIG = {
  baseUrl: getEnvVar('REACT_APP_RAG_API_URL', 'http://localhost:8000'),
  timeout: 30000, // 30 seconds timeout
  maxRetries: 3,
  headers: {
    'Content-Type': 'application/json',
  },
};

// Environment-specific configurations
const ENV_CONFIG = {
  development: {
    baseUrl: getEnvVar('REACT_APP_RAG_API_URL', 'http://localhost:8000'),
  },
  production: {
    baseUrl: getEnvVar('REACT_APP_RAG_API_URL', 'https://api.example.com'), // Replace with actual production URL
  },
  test: {
    baseUrl: getEnvVar('REACT_APP_RAG_API_URL', 'http://localhost:8000'),
  },
};

// Get configuration based on environment
const getApiConfig = () => {
  // Determine environment from Docusaurus custom fields
  let env = getEnvVar('NODE_ENV', 'development');
  const envSpecific = ENV_CONFIG[env as keyof typeof ENV_CONFIG] || ENV_CONFIG.development;

  return {
    ...DEFAULT_API_CONFIG,
    ...envSpecific,
  };
};

export const API_CONFIG = getApiConfig();

// API endpoints
export const API_ENDPOINTS = {
  ask: '/ask',
  health: '/health',
} as const;

// Export the configuration
export default API_CONFIG;