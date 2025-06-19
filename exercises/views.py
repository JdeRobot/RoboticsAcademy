import os
from django.shortcuts import render
from django.http import HttpResponse
from django.conf import settings
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse
from rest_framework.decorators import api_view
from .models import Exercise
from rest_framework.response import Response
from rest_framework import status


# TODO: Too many hardcoded strings, review
def index(request):
    # exercises = Exercise.objects.all()
    exercises = Exercise.objects.all()
    context = {"exercises": exercises}
    return render(request, 'exercises/RoboticsAcademy.html', context)


def load_exercise(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    return render(request, 'exercises/' + exercise_id + '/exercise.html', exercise.context)

@csrf_exempt
@api_view(["POST"])
def user_code_zip(request, exercise_id):
    exercise_path = os.path.join(settings.BASE_DIR, f"exercises/static/exercises/{exercise_id}/python_template/ros2_humble")
    files = []

    try:
        for x in os.listdir(exercise_path):
            with open(os.path.join(exercise_path, x)) as f:
                files.append({"name": x, "content": f.read()})

        return JsonResponse({"success": True, "files": files})

    except Exception as e:
        return Response({"success": False, "message": str(e)}, status=status.HTTP_400_BAD_REQUEST)
