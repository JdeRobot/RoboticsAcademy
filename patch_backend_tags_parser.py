import ast
from pathlib import Path

PATH = Path("/RoboticsAcademy/academy/views.py")
text = PATH.read_text(encoding="utf-8")

if "def parse_tags(raw_tags):" not in text:
    marker = "from django.http import JsonResponse\n"
    helper = (
        "from django.http import JsonResponse\n"
        "import ast\n"
        "\n"
        "def parse_tags(raw_tags):\n"
        "    if not raw_tags:\n"
        "        return []\n"
        "    if isinstance(raw_tags, list):\n"
        "        return raw_tags\n"
        "    if isinstance(raw_tags, tuple):\n"
        "        return list(raw_tags)\n"
        "    if isinstance(raw_tags, str):\n"
        "        raw_tags = raw_tags.strip()\n"
        "        if not raw_tags:\n"
        "            return []\n"
        "        try:\n"
        "            parsed = json.loads(raw_tags)\n"
        "            if isinstance(parsed, list):\n"
        "                return parsed\n"
        "        except Exception:\n"
        "            pass\n"
        "        try:\n"
        "            parsed = ast.literal_eval(raw_tags)\n"
        "            if isinstance(parsed, (list, tuple)):\n"
        "                return list(parsed)\n"
        "        except Exception:\n"
        "            pass\n"
        "    return []\n"
        "\n"
    )
    text = text.replace(marker, helper, 1)

text = text.replace("project_tags = eval(project.tags)", "project_tags = parse_tags(project.tags)")
text = text.replace("tags = eval(project.tags)", "tags = parse_tags(project.tags)")

PATH.write_text(text, encoding="utf-8")
print("patched:", PATH)
