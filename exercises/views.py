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


def get_python_code(request):
    python_code = request.GET.get('python_code', None)
    print("A", python_code)
    if not python_code:
        body_unicode = request.body.decode('utf-8')
        body_unicode = body_unicode[0:18] + body_unicode[18: len(body_unicode) - 2].replace('"',
                                                                                            "'") + body_unicode[-2:]

        body = json.loads(body_unicode, strict=False)

        python_code = body['python_code']
        print("B")
        print(python_code)
    python_code = python_code.lstrip('\\').lstrip('"')
    python_code = python_code.replace('\\n', '\n')
    python_code = python_code.replace('\\"', '"').replace("\\'", "'")
    return python_code

@csrf_exempt
def ros_version(request):    
    output = subprocess.check_output(['bash', '-c', 'echo $ROS_VERSION'])
    output_str = output.decode('utf-8')
    version = output_str[0]
    data = {'version': version}
    return JsonResponse(data)

@csrf_exempt
def launch_files(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    return JsonResponse(data)

# TODO: Too many hardcoded strings, review
def index(request):
    exercises = Exercise.objects.all()
    context = {"exercises": exercises}
    return render(request, 'exercises/RoboticsAcademy.html', context)


def load_exercise(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    return render(request, 'exercises/' + exercise_id + '/exercise.html', exercise.context)


def request_code(request, exercise_id):
    difficulty = request.GET.get('diff')
    path = f'/exercises/static/exercises/{exercise_id}/assets/{difficulty}.py'
    path = str(settings.BASE_DIR) + path
    print('PATH: ', path)
    with open(path, encoding='utf-8') as file:
        data = file.read().replace('\\n', '\n')

    print(data)

    if difficulty is not None:
        print('EXERCISE: ', exercise_id, 'DIFFICULTY: ', difficulty)
        return HttpResponse(data, content_type="text/plain")

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
