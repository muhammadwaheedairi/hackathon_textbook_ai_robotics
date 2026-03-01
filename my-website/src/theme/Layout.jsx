import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/RagChatbot/ChatWidget';
import '@site/src/components/RagChatbot/chat.css';

const Layout = ({ children, ...props }) => {
  return (
    <OriginalLayout {...props}>
      {children}
      <ChatWidget />
    </OriginalLayout>
  );
};

export default Layout;