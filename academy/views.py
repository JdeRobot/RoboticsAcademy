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
from .models import Exercise, Universe, ExerciseUniverses
from .project_view import is_binary_mimetype
from rest_framework.response import Response


@csrf_exempt
@api_view(["GET"])
def save_exercise_db(request):
    """
    Trigger a PostgreSQL dump of exercise-related tables to the repository.

    Dumps tables: exercises, exercises_universes, exercises_tools.
    Output is written to RoboticsAcademy/database/exercises/db.sql.
    """

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
    """
    Trigger a PostgreSQL dump of universe-related tables to a SQL file.

    Dumps tables: universes, worlds, robots, tools.
    Output is written to /universes.sql inside the container.
    """

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


active_project = None


@error_wrapper("GET", ["project_id"])
def enter_exercise(fal, request):
    """
    Retrieve basic information about an exercise. Only called when entering.
    """
    project_id = request.GET.get("project_id")
    project = Exercise.objects.get(exercise_id=project_id)

    tools = []
    for tool in project.tools.all():
        tools.append(tool.name)

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

    # FIX:https://github.com/JdeRobot/RoboticsAcademy/issues/3510
    # global active_project
    # if active_project is None:
    #     active_project = project_id
    # else:
    #     raise Exception("Alredy open session")

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

    base_group = "Code"

    path = fal.exercise_path(project)

    file_list = fal.list_formatted(path, base_group)

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

    if exists_in_helpers(fal, create_path, project_id):
        raise ResourceAlreadyExistsHelpers(create_path)

    fal.create(file_path, content)

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
def get_universes_list(fal, request):
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
def get_docker_universe_data(fal, request):
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

    if not project.universes.exists():
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
