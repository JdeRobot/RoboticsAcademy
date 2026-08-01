"""
API views for Robotics Academy.
"""

import base64
import json
import ast
from django.http import JsonResponse
import subprocess
import sys
from django.views.decorators.csrf import csrf_exempt
from rest_framework.decorators import api_view

from academy.exceptions import (
    ResourceAlreadyExistsHelpers,
)
from academy.project_view import EntryEncoder, exists_in_helpers
from academy.serializers import FileContentSerializer

from .error_handler import error_wrapper
from .templates import select_template
from .models import Exercise, World, ExerciseWorlds, WorldRobots
from .project_view import is_binary_mimetype
from rest_framework.response import Response


@csrf_exempt
@api_view(["GET"])
def save_exercise_db(request):
    """
    Trigger a PostgreSQL dump of exercise-related tables to the repository.

    Dumps tables: exercises, exercises_worlds, exercises_tools.
    Output is written to RoboticsAcademy/database/exercises/db.sql.
    """

    subprocess.Popen(
        [
            """PGPASSWORD="robotics-academy-dev" pg_dump -U user-dev -d academy_db -h world_db --table public.exercises --table public.exercises_worlds --table public.exercises_tools > RoboticsAcademy/database/exercises/db.sql""",
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
def save_world_db(request):
    """
    Trigger a PostgreSQL dump of world-related tables to a SQL file.

    Dumps tables: worlds, scenes, robots, tools.
    Output is written to /worlds.sql inside the container.
    """

    subprocess.Popen(
        [
            """PGPASSWORD="robotics-academy-dev" pg_dump -U user-dev -d academy_db -h world_db --table public.worlds --table public.scenes --table public.robots --table public.tools > /worlds.sql""",
        ],
        shell=True,
        stdout=sys.stdout,
        stderr=subprocess.STDOUT,
        bufsize=1024,
        universal_newlines=True,
    )
    return Response({"success": True})


active_project = None


@error_wrapper("GET", ["project_id"])
def enter_exercise(fal, request):
    """
    Retrieve basic information about an exercise. Only called when entering.
    """
    project_id = request.GET.get("project_id")
    project = Exercise.objects.prefetch_related("tools").get(exercise_id=project_id)

    tools = list(project.tools.values_list("name", flat=True))

    try:
        parsed_tags = ast.literal_eval(project.tags) if project.tags else []
    except (ValueError, SyntaxError):
        parsed_tags = []

    info = {
        "name": project.name,
        "tags": parsed_tags,
        "tools": tools,
        "url": project.url,
    }

    global active_project
    if active_project is None:
        active_project = project_id
    else:
        raise Exception("Already open session")

    # Create filesystem base
    path = fal.exercise_path(project_id)
    if fal.exists(path) < 0:
        fal.mkdir(path)
        # Create base files
        file_path = fal.path_join(path, "academy.py")
        fal.create(file_path, "")
        for tag in parsed_tags:
            if tag == "MULTILANGUAGE":
                file_path = fal.path_join(path, "academy.cpp")
                fal.create(file_path, "")

    return JsonResponse({"success": True, "info": info})


@error_wrapper("POST")
def exit_exercise(fal, request):
    """
    Exit exercise
    """

    global active_project
    active_project = None

    return JsonResponse({"success": True})


@error_wrapper("GET")
def get_exercise_list(fal, request):
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


@error_wrapper("GET", ["project", "language"])
def get_helper_file_list(fal, request):
    """
    Return the list of helper files for a given exercise and language.

    Query params: project (str), language (str).
    """
    project = request.GET.get("project")
    language = request.GET.get("language")

    path = fal.exercise_helper_path(project, language)
    file_list = fal.list_formatted(path, "Code")

    return Response({"file_list": EntryEncoder().encode(file_list)})


@error_wrapper("GET", ["project", "language", "filename"])
def get_helper_file(fal, request):
    """
    Return the content of a specific helper file for an exercise.

    Query params: project (str), language (str), filename (str).
    """
    project = request.GET.get("project")
    language = request.GET.get("language")
    filename = request.GET.get("filename", None)
    binary = request.GET.get("binary", None)

    path = fal.exercise_helper_path(project, language)
    file_path = fal.path_join(path, filename)

    if binary is None or binary is False:
        content = fal.read(file_path)
    else:
        content = fal.read_binary(file_path)
        b64 = base64.b64encode(content)
        content = b64.decode("utf-8")

    serializer = FileContentSerializer({"content": content})
    return Response(serializer.data)


@error_wrapper("GET", ["project"])
def get_file_list(fal, request):
    """
    Return the list of user files for a given exercise project.

    Query params: project (str).
    """
    project = request.GET.get("project")
    user = request.GET.get("user", None)

    base_group = "Code"

    path = fal.exercise_path(project)

    try:
        file_list = fal.list_formatted(path, base_group)
    except Exception:
        return Response({"file_list": EntryEncoder().encode([])})

    return Response({"file_list": EntryEncoder().encode(file_list)})


@error_wrapper("POST", ["project_id", ("location", -1), "file_name"])
def create_file(fal, request):
    """
    Create a new empty file inside the exercise project directory.

    POST params: project_id (str), location (str), file_name (str).
    """
    project_id = request.data.get("project_id")
    location = request.data.get("location")
    filename = request.data.get("file_name")
    template = request.data.get("template", None)

    path = fal.exercise_path(project_id)
    create_path = fal.path_join(location, filename)
    file_path = fal.path_join(path, create_path)

    content = ""

    if template is not None:
        content = select_template(template)

    fal.create(file_path, content)

    if exists_in_helpers(fal, create_path, project_id):
        raise ResourceAlreadyExistsHelpers(create_path)

    return Response({"success": True})


@error_wrapper("POST", ["project_id", ("location", -1), "folder_name"])
def create_folder(fal, request):
    """
    Create a new folder inside the exercise project directory.

    POST params: project_id (str), location (str), folder_name (str).
    """
    project_id = request.data.get("project_id")
    location = request.data.get("location")
    folder_name = request.data.get("folder_name")

    path = fal.exercise_path(project_id)
    create_path = fal.path_join(location, folder_name)
    folder_path = fal.path_join(path, create_path)

    if exists_in_helpers(fal, create_path, project_id, folder=True):
        raise ResourceAlreadyExistsHelpers(create_path)

    fal.mkdir(folder_path)
    return Response({"success": True})


@error_wrapper("POST", ["project_id", "path", "rename_to"])
def rename_file(fal, request):
    """
    Rename a file inside the exercise project directory.

    POST params: project_id (str), path (str), rename_to (str).
    """
    project_id = request.data.get("project_id")
    path = request.data.get("path")
    rename_path = request.data.get("rename_to")

    base_path = fal.exercise_path(project_id)

    if rename_path.startswith("/"):
        rename_path = rename_path[1:]

    file_path = fal.path_join(base_path, path)
    new_path = fal.path_join(base_path, rename_path)

    if exists_in_helpers(fal, rename_path, project_id):
        raise ResourceAlreadyExistsHelpers(rename_path)

    fal.renamefile(file_path, new_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_id", "path", "rename_to"])
def rename_folder(fal, request):
    """
    Rename a folder inside the exercise project directory.

    POST params: project_id (str), path (str), rename_to (str).
    """
    project_id = request.data.get("project_id")
    path = request.data.get("path")
    rename_path = request.data.get("rename_to")

    if rename_path.startswith("/"):
        rename_path = rename_path[1:]

    base_path = fal.exercise_path(project_id)

    file_path = fal.path_join(base_path, path)
    new_path = fal.path_join(base_path, rename_path)

    if exists_in_helpers(fal, rename_path, project_id, folder=True):
        raise ResourceAlreadyExistsHelpers(rename_path)

    fal.renamedir(file_path, new_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_id", "path"])
def delete_file(fal, request):
    """
    Delete a file from the exercise project directory.

    POST params: project_id (str), path (str).
    """
    project_id = request.data.get("project_id")
    path = request.data.get("path")

    base_path = fal.exercise_path(project_id)

    file_path = fal.path_join(base_path, path)

    fal.removefile(file_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_id", "path"])
def delete_folder(fal, request):
    """
    Delete a folder and its contents from the exercise project directory.

    POST params: project_id (str), path (str).
    """
    project_id = request.data.get("project_id")
    path = request.data.get("path")

    base_path = fal.exercise_path(project_id)

    file_path = fal.path_join(base_path, path)

    fal.removedir(file_path)
    return JsonResponse({"success": True})


@error_wrapper("GET", ["project", "filename"])
def get_file(fal, request):
    """
    Return the content of a file from the exercise project directory.

    Query params: project (str), filename (str), binary (bool, optional).
    Returns base64-encoded content if binary=True.
    """
    project_id = request.GET.get("project", None)
    filename = request.GET.get("filename", None)
    user = request.GET.get("user", None)

    binary = request.GET.get("binary", None)

    path = fal.exercise_path(project_id)

    file_path = fal.path_join(path, filename)

    if binary is None or binary is False:
        content = fal.read(file_path)
    else:
        content = fal.read_binary(file_path)
        b64 = base64.b64encode(content)
        content = b64.decode("utf-8")

    serializer = FileContentSerializer({"content": content})
    return Response(serializer.data)


@error_wrapper("POST", ["project", "filename", ("content", -1)])
def save_file(fal, request):
    """
    Save content to a file in the exercise project directory.

    POST params: project (str), filename (str), content (str).
    """
    project_id = request.data.get("project")
    filename = request.data.get("filename")
    content = request.data.get("content")

    path = fal.exercise_path(project_id)

    file_path = fal.path_join(path, filename)

    fal.write(file_path, content)
    return Response({"success": True})


@error_wrapper("GET", ["project"])
def get_worlds_list(fal, request):
    """
    Return the list of worlds associated with an exercise.
    """
    project_id = request.GET.get("project")
    project = Exercise.objects.get(exercise_id=project_id)

    worlds_list = []

    proj_worlds = project.worlds.all()
    proj_worlds = sorted(
        proj_worlds,
        key=lambda world: not ExerciseWorlds.objects.get(
            exercise=project, world=world
        ).is_default,
    )

    for world in proj_worlds:
        worlds_list.append(world.name)

    return Response({"worlds_list": worlds_list})


@error_wrapper("GET", ["project"])
def get_docker_world_data(fal, request):
    """
    Retrieve docker and world configuration for an exercise.
    """
    name = request.GET.get("world")
    project_id = request.GET.get("project")
    project = Exercise.objects.prefetch_related("tools", "worlds").get(
        exercise_id=project_id
    )

    robots = []
    robots_config = []
    tools = []
    tools_configuration = {}

    for tool_name, base_config in project.tools.values_list("name", "base_config"):
        tools.append(tool_name)
        if base_config != "None":
            tools_configuration[tool_name] = base_config

    if not project.worlds.exists():
        config = {
            "name": None,
            "scene": {
                "name": None,
                "launch_file_path": None,
                "ros_version": None,
                "type": None,
                "tools_config": None,
            },
            "robot": robots,
            "tools": tools,
            "tools_config": tools_configuration,
        }
        return Response({"success": True, "world": config})

    world = World.objects.get(name=name)

    ros_version = world.scene.ros_version
    world_type = world.scene.type

    if world.scene.tools_config != "None":
        tools_configuration = json.loads(world.scene.tools_config)

    robot_models = world.robots.all()
    for robot in robot_models:
        for pose in WorldRobots.objects.get(world=world, robot=robot).poses:
            robot_config = {
                "name": robot.name,
                "launch_file_path": robot.launch_file_path,
                "ros_version": ros_version,
                "type": world_type,
                "start_pose": pose,
                "entity": robot.entity,
                "extra_config": robot.extra_config,
            }
            robots_config.append(robot_config.copy())

    config = {
        "name": world.name,
        "scene": {
            "name": world.scene.name,
            "launch_file_path": world.scene.launch_file_path,
            "ros_version": ros_version,
            "type": world_type,
            "tools_config": tools_configuration,
        },
        "robot": robots_config,
        "tools": tools,
        "tools_config": tools_configuration,
    }

    return Response({"success": True, "world": config})


@error_wrapper("POST", ["project_id", "file_name", ("location", -1), "content"])
def upload(fal, request):
    """
    Upload a binary file (e.g. ONNX model) to the exercise project directory.

    POST params: project_id (str), file_name (str), location (str), content (base64 str).
    """
    # Get the name and the zip file from the request
    project_id = request.data.get("project_id")
    file_name = request.data.get("file_name")
    location = request.data.get("location")
    content = request.data.get("content")

    path = fal.exercise_path(project_id)
    create_path = fal.path_join(location, file_name)
    file_path = fal.path_join(path, create_path)

    if exists_in_helpers(fal, create_path, project_id):
        raise ResourceAlreadyExistsHelpers(create_path)

    fal.create_binary(file_path, base64.b64decode(content))
    return Response({"success": True})
