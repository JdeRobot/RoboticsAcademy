import json
import os
import subprocess
import sys
from django.shortcuts import render
from django.http import HttpResponse
from django.conf import settings
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse
from rest_framework.decorators import api_view

from .error_handler import error_wrapper
from .models import Exercise, Universe
from rest_framework.response import Response
from rest_framework import status


# TODO: Too many hardcoded strings, review


def load_exercise(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    return render(request, "react_frontend/exercise.html", exercise.context)


@error_wrapper("POST", ["project", "language"])
def user_code_zip(request):
    project_name = request.data.get("project")
    language = request.data.get("language")
    project = Exercise.objects.get(name=project_name)

    template = "python_template"

    if language == "cpp":
        template = "cpp_template"

    exercise_path = os.path.join(
        settings.BASE_DIR,
        f"exercises/static/exercises/{project.exercise_id}/{template}/ros2_humble",
    )

    print(exercise_path)
    files = []

    try:
        for x in os.listdir(exercise_path):
            new_path = os.path.join(exercise_path, x)
            if os.path.isdir(new_path):
                for y in os.listdir(new_path):
                    with open(os.path.join(new_path, y)) as f:
                        files.append({"name": y, "content": f.read()})
            else:
                with open(new_path) as f:
                    files.append({"name": x, "content": f.read()})

        return JsonResponse({"success": True, "files": files})

    except Exception as e:
        return Response(
            {"success": False, "message": str(e)}, status=status.HTTP_400_BAD_REQUEST
        )


@csrf_exempt
@api_view(["GET"])
def save_exercise_db(request):

    subprocess.Popen(
        [
            """PGPASSWORD="robotics-academy-dev" pg_dump -U user-dev -d academy_db -h universe_db --table public.exercises --table public.exercises_universes --table public.exercises_tools > RoboticsAcademy/database/exercises/db.sql""",
        ],
        shell=True,
        stdout=sys.stdout,
        stderr=subprocess.STDOUT,
        bufsize=1024,
        universal_newlines=True,
    )

    return Response({"success": True})


@csrf_exempt
@api_view(["GET"])
def save_universe_db(request):

    subprocess.Popen(
        [
            """PGPASSWORD="robotics-academy-dev" pg_dump -U user-dev -d academy_db -h universe_db --table public.universes --table public.worlds --table public.robots --table public.tools > /universes.sql""",
        ],
        shell=True,
        stdout=sys.stdout,
        stderr=subprocess.STDOUT,
        bufsize=1024,
        universal_newlines=True,
    )

    return Response({"success": True})


@error_wrapper("GET", ["project"])
def get_universes_list(request):
    project_name = request.GET.get("project")

    project = Exercise.objects.get(name=project_name)
    universes_list = []

    for universe in project.universes.all():
        universes_list.append(universe.name)

    return Response({"universes_list": universes_list})


@error_wrapper("GET", ["project"])
def get_docker_universe_data(request):
    name = request.GET.get("universe")
    project_name = request.GET.get("project")

    project = Exercise.objects.get(name=project_name)

    tools = []
    tools_config = {}
    for tool in project.tools.all():
        tools.append(tool.name)
        if tool.base_config != "None":
            tools_config.update({tool.name: tool.base_config})

    if len(project.universes.all()) == 0:
        config = {
            "name": None,
            "world": {
                "name": None,
                "launch_file_path": None,
                "ros_version": None,
                "type": None,
                "tools_config": None,
            },
            "robot": {
                "name": None,
                "launch_file_path": None,
                "ros_version": None,
                "type": None,
                "start_pose": None,
            },
            "tools": tools,
            "tools_config": tools_config,
        }
    else:
        universe = Universe.objects.get(name=name)

        tools_configuration = None
        if universe.world.tools_config != "None":
            tools_configuration = json.loads(universe.world.tools_config)

        if universe.robot.name != "None":
            robot_config = {
                "name": universe.robot.name,
                "launch_file_path": universe.robot.launch_file_path,
                "ros_version": universe.world.ros_version,
                "type": universe.world.type,
                "start_pose": universe.world.start_pose,
            }
        else:
            robot_config = {
                "name": None,
                "launch_file_path": None,
                "ros_version": None,
                "type": None,
                "start_pose": None,
            }

        config = {
            "name": universe.name,
            "world": {
                "name": universe.world.name,
                "launch_file_path": universe.world.launch_file_path,
                "ros_version": universe.world.ros_version,
                "type": universe.world.type,
                "tools_config": tools_configuration,
            },
            "robot": robot_config,
            "tools": tools,
            "tools_config": tools_configuration,
        }

    # Return the list of projects
    return Response(
        {
            "success": True,
            "universe": config,
        }
    )
