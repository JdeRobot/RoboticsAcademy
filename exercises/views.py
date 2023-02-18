from django.shortcuts import render
from django.http import HttpResponse
from django.conf import settings
from .models import Exercise
import ast
import json

#TODO: Too many hardcoded strings, review
def index(request):
    exercises = Exercise.objects.all()
    context = {"exercises": exercises}
    return render(request, 'exercises/RoboticsAcademy.html', context)


def load_exercise(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    return render(request, 'exercises/' + exercise_id + '/exercise.html', exercise.context)


def request_code(request, exercise_id):
    difficulty = request.GET.get('diff')
    path = '/exercises/static/exercises/{}/web-template/assets/{}.py'.format(exercise_id, difficulty)
    path = str(settings.BASE_DIR) + path
    print('PATH: ', path)
    with open(path) as f:
        data = f.read().replace('\\n', '\n')

    print(data)

    if difficulty != None:
        print('EXERCISE: ', exercise_id, 'DIFFICULTY: ', difficulty)
        return HttpResponse(data, content_type="text/plain")
