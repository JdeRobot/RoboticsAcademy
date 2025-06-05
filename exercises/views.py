import json
import mimetypes
import os
import shutil
import tempfile
import subprocess
import zipfile
import pylint as lint
from django.shortcuts import render
from django.http import HttpResponse
from django.conf import settings
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse
from rest_framework.decorators import api_view
from .models import Exercise
from rest_framework.response import Response
from rest_framework import status
import base64


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
    
    """
    try:
        for x in os.listdir(exercise_path):
            file_path = os.path.join(exercise_path, x)
            with open(file_path, "rb") as f:  # Open in binary mode
                content = f.read()
                # If binary (e.g., .onnx), encode to base64 to make it JSON-safe
                encoded_content = base64.b64encode(content).decode("utf-8")
                files.append({"name": x, "content": encoded_content, "binary": True})

        return JsonResponse({"success": True, "files": files})
    except Exception as e:
        return Response({"success": False, "message": str(e)}, status=status.HTTP_400_BAD_REQUEST)
    """

