---
description: Build modern React functional components with TypeScript, hooks, and best practices.
---

Build modern React functional components with TypeScript, hooks, and best practices.

WHAT IT DOES:
Creates reusable, type-safe React components following modern patterns. Uses hooks, proper state management, error boundaries, accessibility.

COMPONENT PATTERN:
```typescript
import React, { useState, useEffect } from 'react';

interface Props {
  title: string;
  onSubmit: (data: string) => void;
}

export const Component: React.FC<Props> = ({ title, onSubmit }) => {
  const [value, setValue] = useState('');

  const handleSubmit = () => {
    onSubmit(value);
  };

  return (
    <div className="container">
      <h2>{title}</h2>
      <input
        value={value}
        onChange={(e) => setValue(e.target.value)}
        aria-label="Input field"
      />
      <button onClick={handleSubmit}>Submit</button>
    </div>
  );
};
```

USE FOR: Building chat widgets, auth forms, interactive components, UI elements.
