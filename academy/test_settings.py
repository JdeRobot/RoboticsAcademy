"""Lightweight test settings for pytest.

Overrides DATABASES to use an in-memory SQLite database so tests can run
locally without a running Postgres server.
"""
from .settings import *  # noqa: F401,F403

DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.sqlite3",
        "NAME": ":memory:",
    }
}

AUTH_PASSWORD_VALIDATORS = []
