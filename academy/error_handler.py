"""
Utilities for request validation and API error handling.
"""

from rest_framework.response import Response
from rest_framework.decorators import api_view
import binascii
from functools import wraps
import json
import logging
from .exceptions import (
    ActiveSessionExists,
    BinaryNotSupported,
    ResourceAlreadyExistsHelpers,
    ResourceNotExists,
    ResourceAlreadyExists,
    ParameterInvalid,
    InvalidPath,
)
from .file_access import FAL_RA
import os
from django.conf import settings
from copy import copy

CUSTOM_EXCEPTIONS = (
    ActiveSessionExists,
    ResourceNotExists,
    ResourceAlreadyExists,
    ParameterInvalid,
    InvalidPath,
    BinaryNotSupported,
    ResourceAlreadyExistsHelpers,
)

local_fal = FAL_RA(
    settings.BASE_DIR,
    os.path.join(settings.BASE_DIR, "exercises"),
)

logger = logging.getLogger(__name__)


def error_wrapper(type: str, param: list[str | tuple] = []):
    """Decorator for API views with parameter validation and error handling."""

    def decorated(func):
        @wraps(func)
        @api_view([type])
        def wrapper(request):
            try:
                fal = copy(local_fal)
                check_parameters(request.data if type == "POST" else request.GET, param)
                return func(fal, request)
            except CUSTOM_EXCEPTIONS as e:
                logger.warning("API error: %s", e)
                return Response({"message": str(e)}, status=e.error_code)
            except json.JSONDecodeError as e:
                logger.warning("Invalid JSON format: %s", e)
                return Response({"error": f"Invalid JSON format: {str(e)}"}, status=422)
            except (binascii.Error, ValueError) as e:
                logger.warning("Invalid B64 format: %s", e)
                return Response({"error": f"Invalid B64 format: {str(e)}"}, status=422)
            except Exception as e:
                logger.exception("Unhandled error in %s", func.__name__)
                return Response({"error": f"An error occurred: {str(e)}"}, status=500)

        return wrapper

    return decorated


def check_parameters(request, param: list[str | tuple]):
    """Validate required request parameters."""

    for p in param:
        min_len = 0
        if type(p) is tuple:
            min_len = p[1]
            p = p[0]
        if p not in request:
            raise ParameterInvalid(p)
        data = request.get(p)
        if data is None or len(data) <= min_len:
            raise ParameterInvalid(p)
