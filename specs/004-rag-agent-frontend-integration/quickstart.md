# Quickstart: RAG Agent Frontend Integration

## Overview
This guide will help you quickly set up and start using the RAG Agent frontend integration in your Docusaurus textbook site.

## Prerequisites
- Node.js 18+ LTS
- Python 3.11+
- Access to backend RAG API (FastAPI-based)
- Required API keys configured in environment files

## Backend Setup

### 1. Start the RAG Agent API Server
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn api:app --reload --port 8000
```

### 2. Environment Variables
Create a `.env` file in the backend directory with:
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
OPENROUTER_API_KEY=your_openrouter_api_key
```

## Frontend Integration

### 1. Install the RAG Chatbot Component
The component should be added to your Docusaurus site at:
`my-website/src/components/RagChatbot/`

### 2. Component Usage
```jsx
import RagChatbot from '@site/src/components/RagChatbot';

// In your Docusaurus page or layout
function MyPage() {
  return (
    <div>
      <h1>Textbook Content</h1>
      <p>Here is some educational content...</p>

      {/* Add the RAG Chatbot component */}
      <RagChatbot />
    </div>
  );
}
```

### 3. Environment Configuration
Add to your Docusaurus environment (if needed):
```
# In my-website/.env
REACT_APP_RAG_API_URL=http://localhost:8000
```

## Component Features

### User Interface
- **Question Input**: Text area for users to enter their questions
- **Loading State**: Visual indicator during API processing
- **Answer Display**: Shows the AI-generated response
- **Sources Panel**: Lists supporting textbook sources
- **Text Chunks**: Displays matched content from the textbook

### Error Handling
- Network error messages
- Empty response handling
- API timeout management
- Input validation feedback

## API Integration

### Backend Endpoint
- **URL**: `http://localhost:8000/ask` (development) or your deployed backend
- **Method**: POST
- **Content-Type**: application/json
- **Request**: `{ "query": "your question here" }`
- **Response**: QueryResponse object with answer, sources, and matched chunks

### Frontend Service
The component uses a service layer at `my-website/src/services/ragService.js` to handle:
- API communication
- Request/response transformation
- Error handling
- Loading state management

## Development

### Running Locally
1. Start backend: `cd backend && uvicorn api:app --reload --port 8000`
2. Start frontend: `cd my-website && npm run start`
3. Visit `http://localhost:3000` to see the integrated RAG chatbot

### Testing
- Unit tests: `npm test` in the my-website directory
- API integration tests: Check backend test files
- Component tests: Individual component test files

## Deployment

### Environment Variables
Ensure these are set in your deployment environment:
- Backend API URL
- Any required authentication tokens (if applicable)

### Build Process
```bash
cd my-website
npm run build
```

The built site will be available in the `build/` directory and can be served by any static hosting service.

## Troubleshooting

### Common Issues
1. **CORS Errors**: Ensure backend is configured with correct origins
2. **API Not Responding**: Check backend server is running and accessible
3. **Component Not Rendering**: Verify component path and imports

### Debugging
- Check browser console for JavaScript errors
- Verify API endpoint is accessible via browser dev tools
- Confirm environment variables are properly set