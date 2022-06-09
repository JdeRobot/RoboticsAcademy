from rest_framework import viewsets
from rest_framework import permissions

# Create your views here.
from academy.academy_rest_api.serializers.exercises import ExerciseSerializer
from exercises.models import Exercise


class ExerciseViewSet(viewsets.ModelViewSet):
    queryset = Exercise.objects.all()
    serializer_class = ExerciseSerializer
    permission_classes = [permissions.IsAuthenticatedOrReadOnly]
