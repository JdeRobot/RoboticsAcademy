"""
API views for the exercises backend.

Provides endpoints to retrieve exercise data, user templates,
and universe configuration information.
"""

import json
import os
from django.conf import settings
from django.http import JsonResponse

from .error_handler import error_wrapper
from .models import Exercise, Universe, ExerciseUniverses
from rest_framework.response import Response
from rest_framework import status


@error_wrapper("GET", ["project_id"])
def get_info(request):
    """
    Retrieve basic information about an exercise.
    """
    project_id = request.GET.get("project_id")
    project = Exercise.objects.get(exercise_id=project_id)

    tools = []
    for tool in project.tools.all():
        tools.append(tool.name)

    info = {
        "name": project.name,
        "tags": eval(project.tags),
        "tools": tools,
        "url": project.url,
    }

    return JsonResponse({"success": True, "info": info})


@error_wrapper("GET")
def get_exercise_list(request):
    """
    Return a list of all available exercises.
    """
    project_list = []
    projects = Exercise.objects.all()

    for p in projects:
        project_list.append(
            {
                "exercise_id": p.exercise_id,
                "name": p.name,
                "description": p.description,
                "tags": p.tags,
                "status": p.status,
            }
        )

    return JsonResponse({"success": True, "exercises": project_list})


@error_wrapper("POST", ["project", "language"])
def user_code_zip(request):
    """
    Return template source files for a given exercise and language.
    """
    project_id = request.data.get("project")
    language = request.data.get("language")
    project = Exercise.objects.get(exercise_id=project_id)

    template = "python_template"
    if language == "cpp":
        template = "cpp_template"

    exercise_path = os.path.join(
        settings.BASE_DIR,
        f"exercises/static/exercises/{project.exercise_id}/{template}/ros2_humble",
    )

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
            {"success": False, "message": str(e)},
            status=status.HTTP_400_BAD_REQUEST,
        )


@error_wrapper("GET", ["project"])
def get_universes_list(request):
    """
    Return the list of universes associated with an exercise.
    """
    project_id = request.GET.get("project")
    project = Exercise.objects.get(exercise_id=project_id)

    universes_list = []

    proj_univs = project.universes.all()
    proj_univs = sorted(
        proj_univs,
        key=lambda univ: not ExerciseUniverses.objects.get(
            exercise=project, universe=univ
        ).is_default,
    )

    for universe in proj_univs:
        universes_list.append(universe.name)

    return Response({"universes_list": universes_list})


@error_wrapper("GET", ["project"])
def get_docker_universe_data(request):
    """
    Retrieve docker and universe configuration for an exercise.
    """
    name = request.GET.get("universe")
    project_id = request.GET.get("project")
    project = Exercise.objects.get(exercise_id=project_id)

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

    return Response({"success": True, "universe": config})
