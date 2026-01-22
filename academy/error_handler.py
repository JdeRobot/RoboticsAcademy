"""
Utilities for request validation and API error handling.

This module provides a decorator to wrap API views with
common parameter checks and consistent error responses.
"""

from rest_framework.response import Response
from rest_framework.decorators import api_view
import binascii
from functools import wraps
import json
from .exceptions import (
    ResourceNotExists,
    ResourceAlreadyExists,
    ParameterInvalid,
    InvalidPath,
)

CUSTOM_EXCEPTIONS = (
    ResourceNotExists,
    ResourceAlreadyExists,
    ParameterInvalid,
    InvalidPath,
)


def error_wrapper(fal, type: str, param: list[str | tuple] = []):
    """
    Decorator for API views with parameter validation and error handling.

    Args:
        type (str): HTTP method to allow for the view.
        param (list): Required request parameters or (name, min_length) tuples.

    Returns:
        function: Wrapped API view.
    """

    def decorated(func):
        @wraps(func)
        @api_view([type])
        def wrapper(request):
            try:
                check_parameters(request.data if type == "POST" else request.GET, param)
                return func(request)
            except CUSTOM_EXCEPTIONS as e:
                print(str(e))
                return Response({"error": str(e)}, status=e.error_code)
            except json.JSONDecodeError as e:
                print(str(e))
                return Response({"error": f"Invalid JSON format: {str(e)}"}, status=422)
            except (binascii.Error, ValueError) as e:
                print(str(e))
                return Response({"error": f"Invalid B64 format: {str(e)}"}, status=422)
            except Exception as e:
                print(str(e))
                return Response({"error": f"An error occurred: {str(e)}"}, status=500)

        return wrapper

    return decorated


def check_parameters(request, param: list[str | tuple]):
    """
    Validate required request parameters.

    Args:
        request: Request data or query parameters.
        param (list): Required parameters or (name, min_length) tuples.

    Raises:
        ParameterInvalid: If a parameter is missing or invalid.
    """
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
