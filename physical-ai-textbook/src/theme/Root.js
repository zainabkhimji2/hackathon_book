import React from 'react';
import ChatWidget from '../components/ChatWidget';

// Wraps the entire Docusaurus app
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
