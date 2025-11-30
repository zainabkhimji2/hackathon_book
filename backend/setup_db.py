"""Quick database setup script"""
from dotenv import load_dotenv
load_dotenv()

from src.services.db_service import db_service

print("Creating database tables...")
db_service.create_tables()
print("Database tables created successfully!")
