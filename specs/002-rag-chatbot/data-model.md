# Data Model: RAG Chatbot

## Entities

### ChatSession
- **id**: string (UUID) - Unique identifier for the session
- **user_id**: string (optional) - Anonymous user identifier or session token
- **created_at**: datetime - Timestamp when session was created
- **last_activity_at**: datetime - Timestamp of last interaction
- **context_messages**: List[Message] - Conversation history for context
- **is_active**: boolean - Whether session is still active (within 30 min window)

### Message
- **id**: string (UUID) - Unique identifier for the message
- **session_id**: string - Reference to parent ChatSession
- **role**: string (enum: "user", "assistant") - Who sent the message
- **content**: string - The message text
- **timestamp**: datetime - When the message was created
- **metadata**: dict (optional) - Additional message metadata

### ContentChunk
- **id**: string (UUID) - Unique identifier for the chunk
- **content**: string - The text content of the chunk
- **source_document**: string - Reference to the original document
- **source_section**: string (optional) - Section within the document
- **page_number**: integer (optional) - Page reference if applicable
- **embedding_vector**: list[float] - Vector representation for semantic search
- **chunk_metadata**: dict - Additional metadata for retrieval
- **created_at**: datetime - When the chunk was created

### QueryResult
- **chunk**: ContentChunk - The retrieved content chunk
- **similarity_score**: float - Similarity score from vector search (0.0-1.0)
- **rank**: integer - Rank position in results list

### ChatRequest
- **query**: string - User's question or input
- **selected_text**: string (optional) - Text selected by user in the book
- **session_id**: string (optional) - Existing session ID or new session created
- **include_sources**: boolean - Whether to include source information (default: true)

### ChatResponse
- **query**: string - Original user query
- **answer**: string - AI-generated answer
- **results**: List[QueryResult] - Relevant chunks used to generate the answer
- **session_id**: string - Session identifier
- **metadata**: dict - Additional response metadata (timing, model info, etc.)

## Relationships

- **ChatSession** 1:M **Message** (one session contains many messages)
- **ChatSession** 1:1 **User** (one session belongs to one user/anonymous session)
- **Message** M:1 **ChatSession** (many messages belong to one session)

## Validation Rules

### ChatSession
- `last_activity_at` must be >= `created_at`
- Session becomes inactive if `(now - last_activity_at) > 30 minutes`
- `context_messages` should not exceed 20 messages to prevent context overflow

### Message
- `role` must be either "user" or "assistant"
- `content` must not be empty
- `timestamp` must be current or past time

### ContentChunk
- `content` must not be empty
- `embedding_vector` must have consistent dimensionality (4096 for Cohere embed-english-v2.0)
- `similarity_score` must be between 0.0 and 1.0 when used in results

### QueryResult
- `similarity_score` must be between 0.0 and 1.0
- `rank` must be positive integer

## State Transitions

### ChatSession States
1. **New** → **Active**: When first message is sent
2. **Active** → **Inactive**: When 30 minutes pass without activity
3. **Inactive** → **Archived**: After cleanup process (TBD timeline)