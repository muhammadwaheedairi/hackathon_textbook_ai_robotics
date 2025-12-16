/**
 * Search Service for textbook content
 * Provides search functionality with partial matching and debouncing
 */

// Content indexing function to scan textbook pages
const indexContent = () => {
  // This function would typically index all textbook content
  // For now, we'll create a simple in-memory index
  const contentElements = document.querySelectorAll('main p, main h1, main h2, main h3, main h4, main h5, main h6, main li, main td, main th, main div');

  const index = [];
  contentElements.forEach((element, indexPos) => {
    const text = element.textContent || element.innerText;
    if (text.trim() !== '') {
      index.push({
        id: element.id || `text-${indexPos}`,
        content: text,
        element: element,
        url: window.location.href,
        title: document.title
      });
    }
  });

  return index;
};

// Main search function with partial matching
const searchContent = (query, options = {}) => {
  const {
    caseSensitive = false,
    partialMatch = true,
    maxResults = 10
  } = options;

  if (!query || query.trim() === '') {
    return [];
  }

  const searchIndex = indexContent();
  const searchTerm = caseSensitive ? query : query.toLowerCase();
  const results = [];

  searchIndex.forEach(item => {
    const content = caseSensitive ? item.content : item.content.toLowerCase();

    let matchFound = false;
    let matchPositions = [];

    if (partialMatch) {
      // Partial matching using substring search
      const position = content.indexOf(searchTerm);
      if (position !== -1) {
        matchFound = true;
        // Find all occurrences
        let pos = position;
        while (pos !== -1) {
          matchPositions.push(pos);
          pos = content.indexOf(searchTerm, pos + 1);
        }
      }
    } else {
      // Exact matching
      if (content.includes(searchTerm)) {
        matchFound = true;
      }
    }

    if (matchFound) {
      results.push({
        ...item,
        matchPositions,
        matchCount: matchPositions.length,
        snippet: getSnippet(item.content, searchTerm, caseSensitive)
      });
    }
  });

  // Sort results by relevance (more matches first, then by content length)
  results.sort((a, b) => {
    if (b.matchCount !== a.matchCount) {
      return b.matchCount - a.matchCount;
    }
    return a.content.length - b.content.length;
  });

  return results.slice(0, maxResults);
};

// Helper function to get a relevant snippet around the search term
const getSnippet = (content, searchTerm, caseSensitive = false) => {
  const contentToSearch = caseSensitive ? content : content.toLowerCase();
  const searchTermLC = caseSensitive ? searchTerm : searchTerm.toLowerCase();

  const index = contentToSearch.indexOf(searchTermLC);
  if (index === -1) return content.substring(0, 200) + (content.length > 200 ? '...' : '');

  // Get 100 characters before and after the match
  const start = Math.max(0, index - 100);
  const end = Math.min(content.length, index + searchTerm.length + 100);

  let snippet = content.substring(start, end);
  if (start > 0) snippet = '...' + snippet;
  if (end < content.length) snippet = snippet + '...';

  return snippet;
};

// Debounced search function using lodash
const debouncedSearch = (func, delay = 300) => {
  // This would typically use lodash's debounce
  // Since this is a utility file, we'll implement a simple debounce
  let timeoutId;
  return (...args) => {
    clearTimeout(timeoutId);
    timeoutId = setTimeout(() => func.apply(null, args), delay);
  };
};

// Highlight matches in DOM elements
const highlightMatches = (searchTerm, caseSensitive = false) => {
  if (!searchTerm) {
    clearHighlights();
    return;
  }

  // Clear previous highlights
  clearHighlights();

  // Find all text nodes containing the search term
  const walker = document.createTreeWalker(
    document.body,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: function(node) {
        const content = caseSensitive ? node.textContent : node.textContent.toLowerCase();
        const term = caseSensitive ? searchTerm : searchTerm.toLowerCase();
        return content.includes(term) ? NodeFilter.FILTER_ACCEPT : NodeFilter.FILTER_REJECT;
      }
    }
  );

  const textNodes = [];
  let node;
  while (node = walker.nextNode()) {
    textNodes.push(node);
  }

  // Highlight each text node that contains the term
  textNodes.forEach(textNode => {
    const parent = textNode.parentElement;
    if (parent && parent.classList.contains('search-highlight')) {
      // Skip if already highlighted
      return;
    }

    const content = textNode.textContent;
    const term = caseSensitive ? searchTerm : searchTerm.toLowerCase();
    const contentToSearch = caseSensitive ? content : content.toLowerCase();

    const regex = caseSensitive ? new RegExp(`(${searchTerm})`, 'g') : new RegExp(`(${searchTerm})`, 'gi');
    const matches = [...contentToSearch.matchAll(regex)];

    if (matches.length > 0) {
      const fragment = document.createDocumentFragment();
      let lastIndex = 0;

      matches.forEach(match => {
        // Add text before the match
        if (match.index > lastIndex) {
          fragment.appendChild(document.createTextNode(
            content.substring(lastIndex, match.index)
          ));
        }

        // Create highlight element for the match
        const highlight = document.createElement('span');
        highlight.className = 'search-highlight';
        highlight.textContent = content.substring(match.index, match.index + match[0].length);
        highlight.setAttribute('data-original-text', match[0]);
        fragment.appendChild(highlight);

        lastIndex = match.index + match[0].length;
      });

      // Add remaining text after the last match
      if (lastIndex < content.length) {
        fragment.appendChild(document.createTextNode(
          content.substring(lastIndex)
        ));
      }

      // Replace the text node with the fragment containing highlights
      textNode.parentNode.replaceChild(fragment, textNode);
    }
  });
};

// Clear all search highlights
const clearHighlights = () => {
  const highlights = document.querySelectorAll('.search-highlight');
  highlights.forEach(highlight => {
    const originalText = highlight.getAttribute('data-original-text');
    if (originalText) {
      const textNode = document.createTextNode(originalText);
      highlight.parentNode.replaceChild(textNode, highlight);
    }
  });
};

// Handle special characters in search queries
const sanitizeSearchQuery = (query) => {
  // Escape special regex characters
  return query.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
};

// Validate search query length
const validateQuery = (query) => {
  if (!query) return false;
  const trimmed = query.trim();
  return trimmed.length >= 1 && trimmed.length <= 200;
};

// Export the search functions
export {
  searchContent,
  highlightMatches,
  clearHighlights,
  debouncedSearch,
  sanitizeSearchQuery,
  validateQuery,
  indexContent,
  getSnippet
};