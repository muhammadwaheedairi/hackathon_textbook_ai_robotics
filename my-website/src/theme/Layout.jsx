import React, { useState } from 'react';
import OriginalLayout from '@theme-original/Layout';
import SearchBar from '@site/src/components/SearchBar/SearchBar';
import { SearchProvider } from '@site/src/contexts/SearchContext';
import '@site/src/css/search-styles.css';
import '@site/src/components/RagChatbot/chat.css';
import ChatWidget from '@site/src/components/RagChatbot/ChatWidget';

// Custom Layout wrapper to add search functionality and floating chatbot
const Layout = (props) => {
  const [isChatbotVisible, setIsChatbotVisible] = useState(false);

  const toggleChatbot = () => {
    setIsChatbotVisible(!isChatbotVisible);
  };

  const closeChatbot = () => {
    setIsChatbotVisible(false);
  };

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
        <ChatWidget />
      </OriginalLayout>
    </SearchProvider>
  );
};

export default Layout;