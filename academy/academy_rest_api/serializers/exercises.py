from rest_framework import serializers

from exercises.models import Exercise


class ExerciseSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = Exercise
        fields = ['exercise_id', 'name', 'description', 'tags', 'status']



# code format serializer
class CodeFormatSerializer(serializers.Serializer):
    code = serializers.CharField()

# code analysis serializer
class CodeAnalysisSerializer(serializers.Serializer):
    code = serializers.CharField(required=True)
    disable_errors = serializers.ListField(
        child=serializers.CharField(), required=False, default=[]
    )