import React, { useState, useEffect, useRef, useContext } from 'react';
import { debounce } from 'lodash';
import SearchContext from '@site/src/contexts/SearchContext';
import { searchContent, highlightMatches, clearHighlights as clearAllHighlights } from '@site/src/utils/searchService';

/**
 * SearchBar component for textbook content search
 * Implements debounced search with highlighting
 */
const SearchBar = () => {
  const { query, setQuery, results, setResults, isLoading, setLoading, showResults, setShowResults, setSearchTerm } = useContext(SearchContext);
  const inputRef = useRef(null);

  // Debounced search function using the search service
  const debouncedSearch = debounce(async (searchQuery) => {
    if (searchQuery.trim() === '') {
      setResults([]);
      setLoading(false);
      setShowResults(false);
      clearAllHighlights();
      return;
    }

    setLoading(true);

    try {
      // Use the search service to perform the search
      const searchResults = searchContent(searchQuery, { partialMatch: true, maxResults: 10 });
      setResults(searchResults);
      setShowResults(true);

      // Highlight matches in the content
      highlightMatches(searchQuery, false);
    } catch (error) {
      console.error('Search error:', error);
      setResults([]);
    } finally {
      setLoading(false);
    }
  }, 300); // 300ms debounce as specified

  // Handle input changes
  const handleInputChange = (event) => {
    const value = event.target.value;
    setQuery(value);

    // Update search term in context
    setSearchTerm(value);

    // Trigger debounced search
    if (value.trim() !== '') {
      debouncedSearch(value);
    } else {
      setResults([]);
      setShowResults(false);
      clearAllHighlights();
    }
  };

  // Clear search and highlights
  const clearSearch = () => {
    setQuery('');
    setResults([]);
    setShowResults(false);
    clearAllHighlights();
    if (inputRef.current) {
      inputRef.current.focus();
    }
  };

  // Click handler for search results to scroll to content
  const handleResultClick = (id) => {
    const el = document.getElementById(id);
    if (el) {
      el.scrollIntoView({ behavior: 'smooth', block: 'start' });
      // Add visual indication that the element was targeted
      el.classList.add('search-target');
      setTimeout(() => {
        el.classList.remove('search-target');
      }, 2000);
    }
    setShowResults(false);
  };

  // Keyboard navigation for search results
  const handleKeyDown = (event) => {
    if (event.key === 'Escape') {
      setShowResults(false);
      if (inputRef.current) {
        inputRef.current.blur();
      }
    }
  };

  // Handle click outside to close results
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (inputRef.current && !inputRef.current.contains(event.target)) {
        setShowResults(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  return (
    <div className="search-bar-container">
      <div className="search-input-wrapper">
        <input
          ref={inputRef}
          type="text"
          className="search-input"
          placeholder="Search textbook content..."
          value={query}
          onChange={handleInputChange}
          onKeyDown={handleKeyDown}
          onFocus={() => query && setShowResults(true)}
          aria-label="Search textbook content"
          aria-autocomplete="list"
          aria-controls="search-results-list"
          aria-expanded={showResults}
        />
        {query && (
          <button
            className="search-clear-button"
            onClick={clearSearch}
            aria-label="Clear search"
            type="button"
          >
            ×
          </button>
        )}
      </div>

      {showResults && results.length > 0 && (
        <div
          className="search-results-dropdown"
          id="search-results-list"
          role="listbox"
          aria-label="Search results"
        >
          <div className="search-results-header">
            <span>{results.length} result{results.length !== 1 ? 's' : ''} found</span>
          </div>
          <ul className="search-results-list" role="group">
            {results.map((result, index) => (
              <li
                key={result.id || index}
                className="search-result-item"
                role="option"
                aria-selected="false"
              >
                <a
                  href={`#${result.id}`}
                  onClick={(e) => {
                    e.preventDefault();
                    handleResultClick(result.id);
                  }}
                >
                  <div className="result-content">
                    {result.snippet || result.content.substring(0, 100) + (result.content.length > 100 ? '...' : '')}
                  </div>
                  <div className="result-url">{result.title || document.title}</div>
                </a>
              </li>
            ))}
          </ul>
        </div>
      )}

      {showResults && isLoading && (
        <div
          className="search-results-dropdown"
          id="search-results-list"
          role="listbox"
          aria-label="Searching..."
        >
          <div className="search-results-header">
            <span>Searching...</span>
          </div>
        </div>
      )}

      {showResults && results.length === 0 && !isLoading && query && (
        <div
          className="search-results-dropdown"
          id="search-results-list"
          role="listbox"
          aria-label="No results found"
        >
          <div className="search-results-header">
            <span>No results found for "{query}"</span>
          </div>
        </div>
      )}
    </div>
  );
};

export default SearchBar;