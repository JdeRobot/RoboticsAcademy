from json import JSONEncoder
import mimetypes
import chardet
import os


class Entry:
    def __init__(
        self,
        is_dir=False,
        name="",
        path="/",
        group="",
        access=True,
        files=[],
    ):
        self.is_dir = is_dir
        self.name = name
        self.path = path
        self.group = group
        self.access = access
        self.files = files
        self.binary = is_binary_mimetype(name)

    def __str__(self):
        if self.is_dir:
            return self.name + " [%s]" % (", ".join(map(str, self.files)))
        else:
            return self.name


# subclass JSONEncoder
class EntryEncoder(JSONEncoder):
    def default(self, o):
        return o.__dict__


def is_binary_mimetype(file_path):
    mime_type, _ = mimetypes.guess_type(file_path)
    return mime_type is None or mime_type.startswith(
        ("application/", "image/", "video/", "audio/")
    )


def list_dir(base_dir, directory, access_old=True, base_group=""):
    entries = os.listdir(directory)
    values = []
    for entry in entries:
        access = access_old
        group = base_group
        entry_path = os.path.join(directory, entry)
        rel_path = os.path.relpath(entry_path, base_dir)
        if os.path.isfile(entry_path):
            values.append(
                Entry(
                    False,
                    entry,
                    rel_path,
                    group,
                    access,
                )
            )
        else:
            values.append(
                Entry(
                    True,
                    entry,
                    rel_path,
                    group,
                    access,
                    list_dir(base_dir, entry_path, access, group),
                )
            )
    values.sort(key=lambda x: (not x.is_dir, x.name.lower()))
    return values


def check_exist(fal, path, file_list, folder):
    for file in file_list:
        if file.is_dir:
            if folder and path == file.path:
                return True
            check_exist(fal, path, file.files)
        else:
            if not folder and path == file.path:
                return True


def exists_in_helpers(fal, path, project, folder=False):
    for lang in ["python", "cpp"]:
        helper_path = fal.exercise_helper_path(project, lang)
        file_list = fal.list_formatted(helper_path, "Code")
        found = check_exist(fal, path, file_list, folder)
        if found:
            return True
    return False
