from json import JSONEncoder
import os


class Entry:
    def __init__(
        self, is_dir=False, name="", path="/", group="", access=True, files=[]
    ):
        self.is_dir = is_dir
        self.name = name
        self.path = path
        self.group = group
        self.access = access
        self.files = files

    def __str__(self):
        if self.is_dir:
            return self.name + " [%s]" % (", ".join(map(str, self.files)))
        else:
            return self.name


# subclass JSONEncoder
class EntryEncoder(JSONEncoder):
    def default(self, o):
        return o.__dict__


def list_dir(base_dir, directory, access_old=True, base_group=""):
    entries = os.listdir(directory)
    values = []
    for entry in entries:
        access = access_old
        group = base_group
        entry_path = os.path.join(directory, entry)
        rel_path = os.path.relpath(entry_path, base_dir)
        if os.path.isfile(entry_path):
            values.append(Entry(False, entry, rel_path, group, access))
        else:
            if entry == "trees":
                group = "Trees"
                # access = False
            elif entry == "Actions":
                group = "Actions"
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
