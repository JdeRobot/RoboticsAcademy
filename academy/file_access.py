"""File abstraction layer helpers for Robotics Academy."""

from abc import ABC, abstractmethod
import os
import shutil
from .project_view import list_dir
from .exceptions import (
    BinaryNotSupported,
    InvalidPath,
    ResourceNotExists,
    ResourceAlreadyExists,
)


class FAL(ABC):
    """
    Abstract base class defining the File Abstraction Layer (FAL) interface.

    Provides a unified API for file and directory operations used by
    RoboticsAcademy exercises. Concrete implementations handle different
    storage backends (local filesystem, remote, etc.).
    """

    def __init__(self, academy="", helper=""):
        """Initialize backend and helper roots."""
        self.academy = academy
        self.helper = helper
        self.user = None

    def set_user(self, user):
        """Set the current user context for file operations."""
        self.user = user

    @abstractmethod
    def academy_path(self) -> str:
        """Return the root path for all academy exercise files."""
        pass

    def exercise_path(self, exercise_id) -> str:
        """Return the full path for a specific exercise directory."""
        return self.path_join(self.academy_path(), exercise_id)

    def helpers_path(self, exercise_id) -> str:
        """Return the path to the helpers directory for an exercise."""
        return self.path_join(self.helper, exercise_id)

    def exercise_helper_path(self, project_id, language) -> str:
        """Return the template directory for a project language."""
        return self.path_join(
            self.helpers_path(project_id),
            f"{language}_template/",
        )

    @abstractmethod
    def path_join(self, a: str, b: str) -> str:
        """Join two path components and return the result."""
        pass

    @abstractmethod
    def exists(self, path: str) -> int:
        """Return -1 for missing paths, 0 for dirs, or file size for files."""
        pass

    @abstractmethod
    def isdir(self, path: str) -> bool:
        """Return True if path is an existing directory."""
        pass

    @abstractmethod
    def isfile(self, path: str) -> bool:
        """Return True if path is an existing file."""
        pass

    @abstractmethod
    def create(self, path: str, content):
        """Create a text file after validating the path."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) != -1:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def create_binary(self, path: str, content):
        """Create a binary file after validating the path."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) != -1:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def write(self, path: str, content):
        """Overwrite an existing text file."""
        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def write_binary(self, path: str, content):
        """Overwrite an existing binary file."""
        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def read(self, path: str):
        """Read and return text content from a file."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def read_binary(self, path: str):
        """Read and return binary content from a file."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def listdirs(self, path: str):
        """List direct child directories."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def listfiles(self, path: str):
        """List direct child files."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def list_formatted(self, path: str, base_group: str):
        """Return the explorer tree for a directory."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def mkdir(self, path: str):
        """Create a directory."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) >= 0:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def renamefile(self, old_path: str, new_path: str):
        """Rename a file."""
        if ".." in new_path:
            raise InvalidPath(new_path)

        if self.exists(old_path) < 0:
            raise ResourceNotExists(old_path)

        if self.exists(new_path) >= 0:
            raise ResourceAlreadyExists(new_path)

    @abstractmethod
    def renamedir(self, old_path: str, new_path: str):
        """Rename a directory."""
        if ".." in new_path:
            raise InvalidPath(new_path)

        if self.exists(old_path) < 0:
            raise ResourceNotExists(old_path)

        if self.exists(new_path) >= 0:
            raise ResourceAlreadyExists(new_path)

    @abstractmethod
    def removefile(self, path: str):
        """Remove a file."""
        if ".." in path:
            raise InvalidPath(path)

        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

        if not self.isfile(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def removedir(self, path: str):
        """Remove a directory tree."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def dir_size(self, path):
        """Return the total size of a directory tree."""
        if ".." in path:
            raise InvalidPath(path)

        path = self.path_join(path, "")

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    def filename(self, path: str) -> str:
        """Return the filename without extension from a full path."""
        return os.path.splitext(os.path.basename(path))[0]


class FAL_RA(FAL):
    """
    Concrete FAL implementation for local RoboticsAcademy filesystem.

    Stores exercise files under the academy filesystem directory and
    uses standard Python os/shutil operations for all file access.
    """

    def __init__(self, base, helper):
        """Initialize the local filesystem backend."""
        FAL.__init__(self, base, helper)

    def academy_path(self) -> str:
        """Return the local academy workspace root."""
        return self.path_join(self.academy, "filesystem")

    def path_join(self, a: str, b: str) -> str:
        """Join two local filesystem paths."""
        return os.path.join(a, b)

    def exists(self, path: str) -> int:
        """Return -1 for missing paths, 0 for dirs, or file size for files."""
        if not os.path.exists(path):
            return -1

        if os.path.isdir(path):  # It is a dir
            return 0

        return os.path.getsize(path)

    def isdir(self, path: str) -> bool:
        """Return whether a path is a directory."""
        return os.path.isdir(path)

    def isfile(self, path: str) -> bool:
        """Return whether a path is a file."""
        return os.path.isfile(path)

    def create(self, path: str, content):
        """Create a text file."""
        super().create(path, content)

        with open(path, "w") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def create_binary(self, path: str, content):
        """Create a binary file."""
        super().create_binary(path, content)

        with open(path, "wb") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def write(self, path: str, content):
        """Overwrite a text file."""
        super().write(path, content)

        with open(path, "w") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def write_binary(self, path: str, content):
        """Overwrite a binary file."""
        super().write_binary(path, content)

        with open(path, "wb") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def read(self, path: str) -> str:
        """Read text content from a file."""
        super().read(path)

        try:
            with open(path, "r") as f:
                return f.read()
        except Exception:
            raise BinaryNotSupported(path)

    def read_binary(self, path: str) -> bytes:
        """Read binary content from a file."""
        super().read(path)

        with open(path, "rb") as f:
            return f.read()

    def listdirs(self, path: str):
        """List direct child directories."""
        super().listdirs(path)

        return [
            d for d in os.listdir(path)
            if self.isdir(self.path_join(path, d))
        ]

    def listfiles(self, path: str):
        """List direct child files."""
        super().listfiles(path)

        return [
            d for d in os.listdir(path)
            if self.isfile(self.path_join(path, d))
        ]

    def list_formatted(self, path: str, base_group: str):
        """Return the explorer tree for a directory."""
        super().list_formatted(path, base_group)

        # list_dir already returns the tree structure used by the explorer.
        return list_dir(path, path, base_group=base_group)

    def mkdir(self, path: str):
        """Create a directory."""
        super().mkdir(path)

        os.makedirs(path)
        os.chmod(path, mode=0o777)

    def renamefile(self, old_path: str, new_path: str):
        """Rename a file."""
        super().renamefile(old_path, new_path)

        os.rename(old_path, new_path)

    def renamedir(self, old_path: str, new_path: str):
        """Rename a directory."""
        super().renamedir(old_path, new_path)

        os.rename(old_path, new_path)

    def removefile(self, path: str):
        """Remove a file."""
        super().removefile(path)

        os.remove(path)

    def removedir(self, path: str):
        """Remove a directory tree."""
        super().removedir(path)

        shutil.rmtree(path)

    def dir_size(self, path):
        """Return the total size of a directory tree."""
        super().dir_size(path)

        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                # skip if it is symbolic link
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)

        return total_size
