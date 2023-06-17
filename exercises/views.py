import json
import tempfile
import subprocess
from pylint import epylint as lint
from django.shortcuts import render
from django.http import HttpResponse
from django.conf import settings
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse
from .models import Exercise


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

def ros_version(request):    
    output = subprocess.check_output(['bash', '-c', 'echo $ROS_VERSION'])
    output_str = output.decode('utf-8')
    version = output_str[0]
    data = {'version': version}
    return JsonResponse(data)

def launch_files(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    data = json.loads(exercise.configuration)
    return JsonResponse(data)

@csrf_exempt
def evaluate_style(request):
    print("1º")
    try:
        python_code = get_python_code(request)
        code_file = tempfile.NamedTemporaryFile(delete=False)
        code_file.write(python_code.encode())
        code_file.seek(0)
        options = code_file.name + ' --enable=similarities' + " --disable=C0114,C0116"
        (stdout, stderr) = lint.py_run(options, return_std=True)
        code_file.seek(0)
        code_file.close()
        result = stdout.getvalue()
        name = code_file.name.split('/')[-1]
        result = result[(result.find(name) + len(name) - 1):]
        result = result.replace(code_file.name, 'mycode')
        result = result[result.find('\n'):]
        init_index = result.find('rated at ')
        score = -1
        if init_index != -1:
            init_index += len('rated at ')
            final_index = result.find('/10', init_index)
            score = round(float(result[init_index:final_index]), 2)
        response = HttpResponse(result+"\n"+str(score),
                                content_type="text/plain")
        return response
    except Exception as ex:
        print("2º")
        print(ex)
        response = HttpResponse("Error", content_type="text/plain")
        return response


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
