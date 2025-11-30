---
description: Implement secure authentication systems - JWT tokens, password hashing, session management, protected routes.
---

Implement secure authentication systems: JWT tokens, password hashing, session management, protected routes.

WHAT IT DOES:
Creates complete auth systems with signup, signin, session handling, password security, and route protection.

AUTH COMPONENTS:
- Password Hashing: bcrypt with salt rounds
- JWT Generation: Access + refresh tokens
- Token Validation: Verify signature and expiry
- Session Management: HTTP-only cookies
- Protected Routes: Middleware authentication
- Password Reset: Secure token generation

SECURITY CHECKLIST:
✓ Hash passwords with bcrypt (12+ rounds)
✓ Use HTTP-only cookies for tokens
✓ Implement CSRF protection
✓ Rate limit auth endpoints
✓ Validate all inputs
✓ Use secure session storage
✓ Implement token refresh
✓ Log auth events

USE FOR: Building signup/signin, protecting API endpoints, managing user sessions.
