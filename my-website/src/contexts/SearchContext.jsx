import React, { createContext, useContext, useReducer, useEffect } from 'react';

// Create Search Context
const SearchContext = createContext();

// Initial state for search functionality
const initialState = {
  query: '',
  results: [],
  isLoading: false,
  error: null,
  showResults: false,
  searchTerm: '',
  caseSensitive: false,
  partialMatch: true,
  maxResults: 10,
  highlightsVisible: true
};

// Action types
const actionTypes = {
  SET_QUERY: 'SET_QUERY',
  SET_RESULTS: 'SET_RESULTS',
  SET_LOADING: 'SET_LOADING',
  SET_ERROR: 'SET_ERROR',
  SET_SHOW_RESULTS: 'SET_SHOW_RESULTS',
  SET_SEARCH_TERM: 'SET_SEARCH_TERM',
  TOGGLE_CASE_SENSITIVE: 'TOGGLE_CASE_SENSITIVE',
  TOGGLE_PARTIAL_MATCH: 'TOGGLE_PARTIAL_MATCH',
  SET_MAX_RESULTS: 'SET_MAX_RESULTS',
  TOGGLE_HIGHLIGHTS: 'TOGGLE_HIGHLIGHTS',
  CLEAR_SEARCH: 'CLEAR_SEARCH'
};

// Reducer function
const searchReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.SET_QUERY:
      return {
        ...state,
        query: action.payload
      };

    case actionTypes.SET_RESULTS:
      return {
        ...state,
        results: action.payload
      };

    case actionTypes.SET_LOADING:
      return {
        ...state,
        isLoading: action.payload
      };

    case actionTypes.SET_ERROR:
      return {
        ...state,
        error: action.payload
      };

    case actionTypes.SET_SHOW_RESULTS:
      return {
        ...state,
        showResults: action.payload
      };

    case actionTypes.SET_SEARCH_TERM:
      return {
        ...state,
        searchTerm: action.payload
      };

    case actionTypes.TOGGLE_CASE_SENSITIVE:
      return {
        ...state,
        caseSensitive: !state.caseSensitive
      };

    case actionTypes.TOGGLE_PARTIAL_MATCH:
      return {
        ...state,
        partialMatch: !state.partialMatch
      };

    case actionTypes.SET_MAX_RESULTS:
      return {
        ...state,
        maxResults: action.payload
      };

    case actionTypes.TOGGLE_HIGHLIGHTS:
      return {
        ...state,
        highlightsVisible: !state.highlightsVisible
      };

    case actionTypes.CLEAR_SEARCH:
      return {
        ...initialState,
        highlightsVisible: state.highlightsVisible // Preserve highlight visibility setting
      };

    default:
      return state;
  }
};

// Search Provider Component
export const SearchProvider = ({ children }) => {
  const [state, dispatch] = useReducer(searchReducer, initialState);

  // Set query
  const setQuery = (query) => {
    dispatch({ type: actionTypes.SET_QUERY, payload: query });
  };

  // Set results
  const setResults = (results) => {
    dispatch({ type: actionTypes.SET_RESULTS, payload: results });
  };

  // Set loading state
  const setLoading = (isLoading) => {
    dispatch({ type: actionTypes.SET_LOADING, payload: isLoading });
  };

  // Set error
  const setError = (error) => {
    dispatch({ type: actionTypes.SET_ERROR, payload: error });
  };

  // Set show results
  const setShowResults = (showResults) => {
    dispatch({ type: actionTypes.SET_SHOW_RESULTS, payload: showResults });
  };

  // Set search term
  const setSearchTerm = (searchTerm) => {
    dispatch({ type: actionTypes.SET_SEARCH_TERM, payload: searchTerm });
  };

  // Toggle case sensitivity
  const toggleCaseSensitive = () => {
    dispatch({ type: actionTypes.TOGGLE_CASE_SENSITIVE });
  };

  // Toggle partial matching
  const togglePartialMatch = () => {
    dispatch({ type: actionTypes.TOGGLE_PARTIAL_MATCH });
  };

  // Set max results
  const setMaxResults = (maxResults) => {
    dispatch({ type: actionTypes.SET_MAX_RESULTS, payload: maxResults });
  };

  // Toggle highlights visibility
  const toggleHighlights = () => {
    dispatch({ type: actionTypes.TOGGLE_HIGHLIGHTS });
  };

  // Clear search
  const clearSearch = () => {
    dispatch({ type: actionTypes.CLEAR_SEARCH });
  };

  // Context value
  const value = {
    ...state,
    setQuery,
    setResults,
    setLoading,
    setError,
    setShowResults,
    setSearchTerm,
    toggleCaseSensitive,
    togglePartialMatch,
    setMaxResults,
    toggleHighlights,
    clearSearch
  };

  return (
    <SearchContext.Provider value={value}>
      {children}
    </SearchContext.Provider>
  );
};

// Custom hook to use search context
export const useSearch = () => {
  const context = useContext(SearchContext);
  if (!context) {
    throw new Error('useSearch must be used within a SearchProvider');
  }
  return context;
};

export default SearchContext;