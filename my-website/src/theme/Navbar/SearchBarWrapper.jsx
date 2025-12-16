import React from 'react';
import SearchBar from '@site/src/components/SearchBar/SearchBar';

const SearchBarWrapper = () => {
  return (
    <div style={{
      marginLeft: '1rem',
      display: 'flex',
      alignItems: 'center',
      height: '100%'
    }}>
      <SearchBar />
    </div>
  );
};

export default SearchBarWrapper;