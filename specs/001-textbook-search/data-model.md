# Data Model: Textbook Search

## Entities

### Search Query
- **Definition**: The text input by the user in the search bar
- **Fields**:
  - `value` (string): The search term entered by the user
  - `timestamp` (datetime): When the search was initiated
  - `caseSensitive` (boolean): Whether the search should be case-sensitive (default: false)
  - `partialMatch` (boolean): Whether to enable partial matching (default: true)

### Search Results
- **Definition**: Collection of matching content sections within the textbook
- **Fields**:
  - `matches` (array): List of matching content elements
  - `totalCount` (number): Total number of matches found
  - `searchTerm` (string): The original search query that generated these results
  - `timestamp` (datetime): When the results were generated

### Match Item
- **Definition**: A single matching occurrence in the content
- **Fields**:
  - `content` (string): The text content that contains the match
  - `highlightedContent` (string): The content with highlighted search terms
  - `location` (object): Position information (element ID, page, section)
  - `score` (number): Relevance score (for future enhancement)

### Highlighted Content
- **Definition**: Textbook content with search term matches visually emphasized
- **Fields**:
  - `originalContent` (string): The unmodified content
  - `highlightedHtml` (string): HTML with highlighted spans
  - `matchCount` (number): Number of highlighted terms in the content

## Relationships
- Search Query → Search Results (one-to-many)
- Search Results → Match Item (one-to-many)
- Match Item → Highlighted Content (one-to-one)

## Validation Rules
- Search query must be 1-200 characters
- Search query cannot be empty or whitespace-only
- Search results must be cleared when query is cleared
- Highlighted content must preserve original formatting and links

## State Transitions
- Search Query: `idle` → `typing` → `searching` → `results_displayed`
- Search Results: `empty` → `searching` → `populated` → `cleared`