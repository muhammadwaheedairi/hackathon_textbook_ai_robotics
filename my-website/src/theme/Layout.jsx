import React, { useState } from 'react';
import OriginalLayout from '@theme-original/Layout';
import '@site/src/components/RagChatbot/chat.css';
import ChatWidget from '@site/src/components/RagChatbot/ChatWidget';

// Custom Layout wrapper to add floating chatbot
const Layout = (props) => {
  const [isChatbotVisible, setIsChatbotVisible] = useState(false);

  const toggleChatbot = () => {
    setIsChatbotVisible(!isChatbotVisible);
  };

  const closeChatbot = () => {
    setIsChatbotVisible(false);
  };

  return (
    <OriginalLayout {...props}>
      {props.children}
      <ChatWidget />
    </OriginalLayout>
  );
};

export default Layout;