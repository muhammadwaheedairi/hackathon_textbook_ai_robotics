# Data Model: RAG Agent Frontend Integration

## Entities

### Question
- **Description**: User input text representing an inquiry about textbook content
- **Fields**:
  - `query: string` - The user's question text (required, 1-2000 characters)
- **Validation**:
  - Cannot be empty or contain only whitespace
  - Maximum length of 2000 characters
- **State Transitions**: N/A (immutable input)

### Answer
- **Description**: AI-generated response to the user's question based on textbook content
- **Fields**:
  - `answer: string` - The generated answer text (required)
  - `status: "success" | "error" | "empty"` - Processing status (required)
  - `error: string?` - Error message if processing failed (optional)
  - `query_time_ms: number?` - Processing time in milliseconds (optional)
  - `confidence: string?` - Confidence level indicator (optional)
- **Validation**:
  - Answer text must be present when status is "success"
  - Error message must be present when status is "error"
- **State Transitions**: Pending → Processed (success/error)

### Supporting Sources
- **Description**: References to specific locations in the textbook that support the answer
- **Fields**:
  - `sources: string[]` - Array of URL strings pointing to relevant textbook sections
- **Validation**:
  - Each URL must be properly formatted
  - Array can be empty if no sources found
- **Relationships**: Associated with an Answer entity

### Text Chunk
- **Description**: Specific passages from the textbook that were used as context for generating the answer
- **Fields**:
  - `content: string` - The actual text content of the chunk (required)
  - `url: string` - URL to the source document (required)
  - `position: number` - Position index of the chunk within the source (required)
  - `similarity_score: number` - Relevance score from the retrieval system (required)
- **Validation**:
  - Content must be non-empty
  - URL must be properly formatted
  - Position must be non-negative integer
  - Similarity score must be between 0 and 1
- **Relationships**: Associated with an Answer entity as part of `matched_chunks` array

## Frontend State Model

### UI State
- **Description**: Represents the current state of the RAG chatbot UI
- **Fields**:
  - `query: string` - Current user input (default: "")
  - `isLoading: boolean` - Whether a request is currently processing (default: false)
  - `answer: Answer?` - The current answer response (default: null)
  - `error: string?` - Any error message to display (default: null)
  - `showResults: boolean` - Whether to display the results panel (default: false)
- **State Transitions**:
  - Initial → Input (user types query)
  - Input → Loading (user submits query)
  - Loading → Success/Error (response received)

## API Contract Model

### Request Model
- **QueryRequest** (Backend API input)
  - `query: string` - The question text to be answered

### Response Model
- **QueryResponse** (Backend API output)
  - `answer: string` - Generated answer text
  - `sources: string[]` - Array of source URLs
  - `matched_chunks: TextChunk[]` - Array of relevant text chunks
  - `error: string?` - Error message if applicable
  - `status: string` - Status of the request ("success", "error", "empty")
  - `query_time_ms: number?` - Processing time
  - `confidence: string?` - Confidence level

## Component Data Flow

### Frontend Component Props
- `initialQuery?: string` - Optional initial query value
- `onSubmit?: (query: string) => void` - Callback when query is submitted
- `onResponse?: (response: Answer) => void` - Callback when response is received

### Internal Component State
- Maintains query text, loading state, response data, and error states
- Manages UI visibility states (show/hide results panel)
- Handles user interactions and API communication