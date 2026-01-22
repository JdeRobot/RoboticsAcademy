# File Abstraction Layer

from abc import ABC, abstractmethod
import os
import shutil
from .project_view import list_dir
from .exceptions import InvalidPath, ResourceNotExists, ResourceAlreadyExists


class FAL(ABC):
    """File Abstraction Layer"""

    def __init__(self, academy="", helper=""):
        self.academy = academy
        self.helper = helper
        self.user = None

    def set_user(self, user):
        self.user = user

    @abstractmethod
    def academy_path(self) -> str:
        pass

    def exercise_path(self, exercise_id) -> str:
        return self.path_join(self.academy_path(), exercise_id)

    def helpers_path(self, exercise_id) -> str:
        return self.path_join(self.helper, exercise_id)

    def exercise_helper_path(self, project_id, language) -> str:
        return self.path_join(self.helpers_path(project_id), f"{language}_template/")

    @abstractmethod
    def path_join(self, a: str, b: str) -> str:
        pass

    @abstractmethod
    def exists(self, path: str) -> bool:
        pass

    @abstractmethod
    def isdir(self, path: str) -> bool:
        pass

    @abstractmethod
    def isfile(self, path: str) -> bool:
        pass

    @abstractmethod
    def create(self, path: str, content):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) > 0:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def create_binary(self, path: str, content):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) > 0:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def write(self, path: str, content):
        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def write_binary(self, path: str, content):
        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def read(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def read_binary(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def listdirs(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def listfiles(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def list_formatted(self, path: str, base_group: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def mkdir(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) >= 0:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def renamefile(self, old_path: str, new_path: str):
        if ".." in new_path:
            raise InvalidPath(new_path)

        if self.exists(old_path) < 0:
            raise ResourceNotExists(old_path)

        if self.exists(new_path) >= 0:
            raise ResourceAlreadyExists(new_path)

    @abstractmethod
    def renamedir(self, old_path: str, new_path: str):
        if ".." in new_path:
            raise InvalidPath(new_path)

        if self.exists(old_path) < 0:
            raise ResourceNotExists(old_path)

        if self.exists(new_path) >= 0:
            raise ResourceAlreadyExists(new_path)

    @abstractmethod
    def removefile(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

        if not self.isfile(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def removedir(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def dir_size(self, path):
        if ".." in path:
            raise InvalidPath(path)

        path = self.path_join(path, "")

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    def filename(self, path: str) -> str:
        return os.path.splitext(os.path.basename(path))[0]


class FAL_RA(FAL):
    """File Abstraction Layer"""

    def __init__(self, base, helper):
        FAL.__init__(self, base, helper)

    def academy_path(self) -> str:
        return self.path_join(self.academy, "filesystem")

    def path_join(self, a: str, b: str) -> str:
        return os.path.join(a, b)

    def exists(self, path: str) -> bool:
        if not os.path.exists(path):
            return -1

        if os.path.isdir(path):  # It is a dir
            return 0

        return os.path.getsize(path)

    def isdir(self, path: str) -> bool:
        return os.path.isdir(path)

    def isfile(self, path: str) -> bool:
        return os.path.isfile(path)

    def create(self, path: str, content):
        super().create(path, content)

        with open(path, "w") as f:
            f.write(content)

    def create_binary(self, path: str, content):
        super().create_binary(path, content)

        with open(path, "wb") as f:
            f.write(content)

    def write(self, path: str, content):
        super().write(path, content)

        with open(path, "w") as f:
            f.write(content)

    def write_binary(self, path: str, content):
        super().write_binary(path, content)

        with open(path, "wb") as f:
            f.write(content)

    def read(self, path: str) -> str:
        super().read(path)

        with open(path, "r") as f:
            return f.read()

    def read_binary(self, path: str) -> str:
        super().read(path)

        with open(path, "rb") as f:
            return f.read()

    def listdirs(self, path: str):
        super().listdirs(path)

        return [d for d in os.listdir(path) if self.isdir(self.path_join(path, d))]

    def listfiles(self, path: str):
        super().listfiles(path)

        return [d for d in os.listdir(path) if self.isfile(self.path_join(path, d))]

    def list_formatted(self, path: str, base_group: str):
        super().list_formatted(path, base_group)

        return list_dir(path, path, base_group=base_group)

    def mkdir(self, path: str):
        super().mkdir(path)

        os.makedirs(path)

    def renamefile(self, old_path: str, new_path: str):
        super().renamefile(old_path.new_path)

        os.rename(old_path, new_path)

    def renamedir(self, old_path: str, new_path: str):
        super().renamedir(old_path.new_path)

        os.rename(old_path, new_path)

    def removefile(self, path: str):
        super().removefile(path)

        os.remove(path)

    def removedir(self, path: str):
        super().removedir(path)

        shutil.rmtree(path)

    def dir_size(self, path):
        super().dir_size(path)

        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                # skip if it is symbolic link
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)

        return total_size
