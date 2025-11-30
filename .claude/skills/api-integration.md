---
description: Connect frontends to backend APIs with proper error handling, loading states, and type safety.
---

Connect frontends to backend APIs with proper error handling, loading states, and type safety.

WHAT IT DOES:
Implements API calls from React to FastAPI with TypeScript types, error handling, loading indicators, and retry logic.

INTEGRATION PATTERN:
```typescript
const [loading, setLoading] = useState(false);
const [error, setError] = useState<string | null>(null);

const fetchData = async () => {
  setLoading(true);
  setError(null);
  try {
    const response = await fetch('/api/endpoint', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    });
    if (!response.ok) throw new Error('Request failed');
    const result = await response.json();
    return result;
  } catch (err) {
    setError(err.message);
  } finally {
    setLoading(false);
  }
};
```

USE FOR: Connecting chat widget to backend, auth API calls, data fetching.
