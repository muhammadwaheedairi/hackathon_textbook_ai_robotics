# Search Feature Documentation

## Overview

The textbook search feature provides a powerful search capability that allows users to quickly find content within the textbook. The search functionality includes highlighting of matching terms, partial matching support, and debounced real-time updates for optimal performance.

## Features

### 1. Real-time Search with Debouncing

The search updates results as you type with a 300ms debounce to optimize performance and prevent excessive processing during typing.

### 2. Partial Matching

The search supports partial matching, allowing users to find content even when they don't remember the exact wording. For example, searching for "search" will match content containing "searching" or "searchable".

### 3. Visual Highlighting

Matching terms in the content are visually highlighted with a yellow background to help users quickly identify relevant sections.

### 4. Responsive Design

The search bar is fully responsive and works well on both desktop and mobile devices.

### 5. Accessibility

The search functionality includes proper ARIA labels, keyboard navigation, and screen reader support.

## Usage

### Basic Search

1. Type your search term in the search bar at the top of the page
2. Results will appear automatically after a brief delay (debounced)
3. Matching terms in the content will be highlighted

### Keyboard Navigation

- `Escape` - Close search results dropdown
- Search results can be navigated with keyboard for accessibility

### Search Tips

- The search is case-insensitive by default
- Partial matches are enabled by default
- Special characters are handled appropriately
- Long search queries (over 200 characters) are not allowed

## API and Component Interface

### Search Service Functions

The search functionality is powered by the `searchService.js` utility which provides the following functions:

#### `searchContent(query, options)`
- **Description**: Performs search on indexed content
- **Parameters**:
  - `query` (string): The search term
  - `options` (object): Search options
    - `caseSensitive` (boolean): Whether to perform case-sensitive matching (default: false)
    - `partialMatch` (boolean): Whether to enable partial matching (default: true)
    - `maxResults` (number): Maximum number of results to return (default: 10)
- **Returns**: Array of matching content items

#### `highlightMatches(searchTerm, caseSensitive)`
- **Description**: Highlights matching terms in the DOM
- **Parameters**:
  - `searchTerm` (string): The term to highlight
  - `caseSensitive` (boolean): Whether matching should be case-sensitive (default: false)

#### `clearHighlights()`
- **Description**: Removes all search highlights from the page

#### `validateQuery(query)`
- **Description**: Validates a search query
- **Parameters**:
  - `query` (string): The query to validate
- **Returns**: Boolean indicating if query is valid (1-200 characters)

#### `sanitizeSearchQuery(query)`
- **Description**: Sanitizes a search query by escaping special regex characters
- **Parameters**:
  - `query` (string): The query to sanitize
- **Returns**: Sanitized query string

## Performance Characteristics

- **Search Update Time**: Results update within 500ms after typing stops
- **Debounce Delay**: 300ms to balance responsiveness and performance
- **Maximum Query Length**: 200 characters
- **Performance Degradation**: Less than 10% compared to baseline
- **P95 Search Time**: Less than 200ms

## Integration

The search bar is integrated into the Docusaurus theme layout and appears above textbook content pages. It uses the SearchContext for state management and the search service for functionality.

## Troubleshooting

### Search is not working
- Ensure JavaScript is enabled in your browser
- Clear browser cache and try again
- Check browser console for any JavaScript errors

### Highlights are not appearing
- Make sure you're searching for content that exists on the current page
- Try clearing and re-entering your search term

### Performance seems slow
- The search is debounced to 300ms by default for performance
- Very long search queries may take longer to process
- Results are cached after the initial search

## Accessibility Features

- ARIA labels for search input and results
- Keyboard navigation support
- Screen reader compatibility
- Proper focus management
- Sufficient color contrast for highlights

## Security Considerations

- Search queries are processed client-side only
- No user data is transmitted to external services
- Special characters are properly sanitized to prevent injection
- Content indexing happens locally in the browser

## Technical Implementation

### Architecture

The search functionality is implemented using:
- React components for the UI
- Context API for state management
- Client-side JavaScript for search and highlighting
- CSS for styling and animations

### File Structure

```
src/
├── components/
│   └── SearchBar/
│       └── SearchBar.jsx
├── utils/
│   └── searchService.js
├── contexts/
│   └── SearchContext.jsx
└── css/
    └── search-styles.css
```

## Limitations

- Search only works with content currently loaded in the browser
- Dynamic content loaded after page load may not be indexed
- Search is limited to the current page's content
- Maximum query length is 200 characters