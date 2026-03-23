"""
Serializers for Robotics Academy API.
"""

from rest_framework import serializers


class FileContentSerializer(serializers.Serializer):
    """
    Serializer for file content responses.

    Fields:
        content: The text content of the file as a string.
    """

    content = serializers.CharField()
