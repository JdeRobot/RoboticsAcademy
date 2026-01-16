import subprocess
import sys
from django.views.decorators.csrf import csrf_exempt
from rest_framework.decorators import api_view
from rest_framework.response import Response


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
