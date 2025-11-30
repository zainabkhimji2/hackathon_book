---
description: Design efficient, normalized database schemas with proper relationships, indexes, and constraints.
---

Design efficient, normalized database schemas with proper relationships, indexes, and constraints.

WHAT IT DOES:
Creates PostgreSQL schemas following normalization principles, defines relationships, adds indexes for performance, ensures data integrity.

SCHEMA ELEMENTS:
- Primary keys (UUID recommended)
- Foreign keys with proper constraints
- Indexes on query columns
- Timestamps (created_at, updated_at)
- NOT NULL constraints where appropriate
- UNIQUE constraints for business rules
- Check constraints for validation

EXAMPLE:
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);
```

USE FOR: Designing all database tables, creating schemas, planning data models.
