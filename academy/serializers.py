from rest_framework import serializers


class FileContentSerializer(serializers.Serializer):
    content = serializers.CharField()
