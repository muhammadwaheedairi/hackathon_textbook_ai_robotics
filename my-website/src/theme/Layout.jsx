import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import SearchBar from '@site/src/components/SearchBar/SearchBar';
import { SearchProvider } from '@site/src/contexts/SearchContext';
import '@site/src/css/search-styles.css';

// Custom Layout wrapper to add search functionality
const Layout = (props) => {
  return (
    <SearchProvider>
      <OriginalLayout {...props}>
        {/* Add search bar above the main content on docs pages */}
        <div className="search-bar-wrapper">
          <div style={{
            width: '100%',
            display: 'flex',
            justifyContent: 'center',
            padding: '1rem 0',
            backgroundColor: 'var(--ifm-background-surface-color)',
            borderBottom: '1px solid var(--ifm-toc-border-color)'
          }}>
            <div style={{ width: '100%', maxWidth: '1200px', paddingLeft: '1rem', paddingRight: '1rem' }}>
              <SearchBar />
            </div>
          </div>
        </div>
        {props.children}
      </OriginalLayout>
    </SearchProvider>
  );
};

export default Layout;