"""
Unit tests for Robotics Academy API views.

Tests cover parameter validation, error handling, and core
file management operations using Django's test client.
"""

import base64
import os
import shutil
import tempfile

from django.test import TestCase
from django.urls import reverse

from academy.exceptions import (
    InvalidPath,
    ParameterInvalid,
    ResourceAlreadyExists,
    ResourceNotExists,
)
from academy.file_access import FAL_RA
from academy.models import Exercise, Universe, World, Robot, Tool
from academy.templates import select_template

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_exercise(exercise_id="test_ex", name="Test Exercise"):
    """Create and return a minimal Exercise instance for testing."""
    return Exercise.objects.create(
        exercise_id=exercise_id,
        name=name,
        description="A test exercise",
        tags="[]",
        status="ACTIVE",
    )


# ---------------------------------------------------------------------------
# FAL_RA Unit Tests
# ---------------------------------------------------------------------------


class FALRATests(TestCase):
    """Unit tests for the FAL_RA filesystem abstraction layer."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.fal = FAL_RA(self.tmp, self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_path_join(self):
        result = self.fal.path_join("/a", "b")
        self.assertEqual(result, "/a/b")

    def test_exists_missing(self):
        self.assertEqual(self.fal.exists("/nonexistent/path"), -1)

    def test_exists_directory(self):
        self.assertEqual(self.fal.exists(self.tmp), 0)

    def test_exists_file(self):
        path = os.path.join(self.tmp, "sample.txt")
        with open(path, "w") as f:
            f.write("hello")
        self.assertGreater(self.fal.exists(path), 0)

    def test_isdir_true(self):
        self.assertTrue(self.fal.isdir(self.tmp))

    def test_isdir_false_for_file(self):
        path = os.path.join(self.tmp, "file.txt")
        open(path, "w").close()
        self.assertFalse(self.fal.isdir(path))

    def test_isfile_true(self):
        path = os.path.join(self.tmp, "file.txt")
        open(path, "w").close()
        self.assertTrue(self.fal.isfile(path))

    def test_create_and_read(self):
        path = os.path.join(self.tmp, "new.py")
        self.fal.create(path, "print('hello')")
        self.assertEqual(self.fal.read(path), "print('hello')")

    def test_create_raises_if_exists(self):
        path = os.path.join(self.tmp, "exists.py")
        self.fal.create(path, "")
        with self.assertRaises(ResourceAlreadyExists):
            self.fal.create(path, "")

    def test_create_raises_on_path_traversal(self):
        with self.assertRaises(InvalidPath):
            self.fal.create(os.path.join(self.tmp, "../evil.py"), "")

    def test_write_updates_content(self):
        path = os.path.join(self.tmp, "update.py")
        self.fal.create(path, "old")
        self.fal.write(path, "new")
        self.assertEqual(self.fal.read(path), "new")

    def test_write_raises_if_missing(self):
        with self.assertRaises(ResourceNotExists):
            self.fal.write(os.path.join(self.tmp, "ghost.py"), "data")

    def test_create_binary_and_read_binary(self):
        path = os.path.join(self.tmp, "model.onnx")
        self.fal.create_binary(path, b"\x00\x01\x02")
        self.assertEqual(self.fal.read_binary(path), b"\x00\x01\x02")

    def test_mkdir_and_listdirs(self):
        subdir = os.path.join(self.tmp, "subdir")
        self.fal.mkdir(subdir)
        self.assertIn("subdir", self.fal.listdirs(self.tmp))

    def test_mkdir_raises_if_exists(self):
        with self.assertRaises(ResourceAlreadyExists):
            self.fal.mkdir(self.tmp)

    def test_listfiles(self):
        path = os.path.join(self.tmp, "listed.py")
        self.fal.create(path, "")
        self.assertIn("listed.py", self.fal.listfiles(self.tmp))

    def test_removefile(self):
        path = os.path.join(self.tmp, "remove_me.py")
        self.fal.create(path, "")
        self.fal.removefile(path)
        self.assertEqual(self.fal.exists(path), -1)

    def test_removefile_raises_if_missing(self):
        with self.assertRaises(ResourceNotExists):
            self.fal.removefile(os.path.join(self.tmp, "ghost.py"))

    def test_removedir(self):
        subdir = os.path.join(self.tmp, "to_remove")
        self.fal.mkdir(subdir)
        self.fal.removedir(subdir)
        self.assertEqual(self.fal.exists(subdir), -1)

    def test_renamefile(self):
        old = os.path.join(self.tmp, "old.py")
        new = os.path.join(self.tmp, "new.py")
        self.fal.create(old, "content")
        self.fal.renamefile(old, new)
        self.assertTrue(self.fal.isfile(new))
        self.assertEqual(self.fal.exists(old), -1)

    def test_dir_size(self):
        path = os.path.join(self.tmp, "sized.txt")
        self.fal.create(path, "12345")
        size = self.fal.dir_size(self.tmp)
        self.assertGreater(size, 0)

    def test_filename(self):
        self.assertEqual(self.fal.filename("/some/path/file.py"), "file")

    def test_read_raises_on_path_traversal(self):
        with self.assertRaises(InvalidPath):
            self.fal.read(os.path.join(self.tmp, "../etc/passwd"))


# ---------------------------------------------------------------------------
# Exception Unit Tests
# ---------------------------------------------------------------------------


class ExceptionTests(TestCase):
    """Unit tests for custom exception classes."""

    def test_invalid_path_message(self):
        exc = InvalidPath("/bad/../path")
        self.assertIn("Path:", str(exc))
        self.assertEqual(exc.error_code, 403)

    def test_parameter_invalid_message(self):
        from academy.exceptions import ParameterInvalid

        exc = ParameterInvalid("project_id")
        self.assertIn("project_id", str(exc))
        self.assertEqual(exc.error_code, 400)

    def test_resource_not_exists_message(self):
        exc = ResourceNotExists("/missing/file.py")
        self.assertIn("does not exist", str(exc))
        self.assertEqual(exc.error_code, 404)

    def test_resource_already_exists_message(self):
        exc = ResourceAlreadyExists("/existing/file.py")
        self.assertIn("already exists", str(exc))
        self.assertEqual(exc.error_code, 409)

    def test_binary_not_supported_message(self):
        from academy.exceptions import BinaryNotSupported

        exc = BinaryNotSupported("/some/file.bin")
        self.assertIn("not supported", str(exc))
        self.assertEqual(exc.error_code, 415)


# ---------------------------------------------------------------------------
# API View Tests
# ---------------------------------------------------------------------------


class GetExerciseListViewTests(TestCase):
    """Tests for the get_exercise_list API endpoint."""

    def test_returns_empty_list_when_no_exercises(self):
        response = self.client.get("/academy/get_exercise_list/")
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertTrue(data["success"])
        self.assertEqual(data["exercises"], [])

    def test_returns_exercise_data(self):
        _make_exercise("follow_line", "Follow Line")
        response = self.client.get("/academy/get_exercise_list/")
        self.assertEqual(response.status_code, 200)
        exercises = response.json()["exercises"]
        self.assertEqual(len(exercises), 1)
        self.assertEqual(exercises[0]["exercise_id"], "follow_line")
        self.assertEqual(exercises[0]["name"], "Follow Line")

    def test_post_method_not_allowed(self):
        response = self.client.post("/academy/get_exercise_list/")
        self.assertEqual(response.status_code, 405)


class EnterExerciseViewTests(TestCase):
    """Tests for the enter_exercise API endpoint."""

    def test_missing_project_id_returns_400(self):
        response = self.client.get("/academy/enter_exercise/")
        self.assertEqual(response.status_code, 400)

    def test_nonexistent_exercise_returns_500(self):
        response = self.client.get(
            "/academy/enter_exercise/", {"project_id": "nonexistent"}
        )
        self.assertEqual(response.status_code, 500)

    def test_valid_exercise_returns_success(self):
        import tempfile, shutil
        from academy import error_handler
        from academy.file_access import FAL_RA

        tmp = tempfile.mkdtemp()
        orig = error_handler.local_fal
        error_handler.local_fal = FAL_RA(tmp, os.path.join(tmp, "exercises"))
        try:
            _make_exercise("test_enter", "Test Enter")
            response = self.client.get(
                "/academy/enter_exercise/", {"project_id": "test_enter"}
            )
            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertTrue(data["success"])
            self.assertIn("info", data)
            self.assertEqual(data["info"]["name"], "Test Enter")
        finally:
            error_handler.local_fal = orig
            shutil.rmtree(tmp, ignore_errors=True)


class FileManagementViewTests(TestCase):
    """Tests for file management API endpoints."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.exercise = _make_exercise("file_test_ex", "File Test Exercise")
        # Pre-create the exercise directory as enter_exercise would
        ex_dir = os.path.join(self.tmp, "filesystem", "file_test_ex")
        os.makedirs(ex_dir, exist_ok=True)
        with open(os.path.join(ex_dir, "academy.py"), "w") as f:
            f.write("# starter")
        # Patch local_fal directly since it is instantiated at import time
        from academy import error_handler
        from academy.file_access import FAL_RA

        self._orig_fal = error_handler.local_fal
        error_handler.local_fal = FAL_RA(
            self.tmp,
            os.path.join(self.tmp, "exercises"),
        )

    def tearDown(self):
        from academy import error_handler

        error_handler.local_fal = self._orig_fal
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_get_file_list_missing_project_returns_400(self):
        response = self.client.get("/academy/get_file_list/")
        self.assertEqual(response.status_code, 400)

    def test_get_file_list_valid_project(self):
        response = self.client.get(
            "/academy/get_file_list/", {"project": "file_test_ex"}
        )
        self.assertEqual(response.status_code, 200)

    def test_save_file_missing_params_returns_400(self):
        response = self.client.post("/academy/save_file/", {"project": "file_test_ex"})
        self.assertEqual(response.status_code, 400)

    def test_save_and_get_file(self):
        # Save content to academy.py
        response = self.client.post(
            "/academy/save_file/",
            {
                "project": "file_test_ex",
                "filename": "academy.py",
                "content": "print('hello')",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json()["success"])

        # Retrieve it back
        response = self.client.get(
            "/academy/get_file/",
            {"project": "file_test_ex", "filename": "academy.py"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["content"], "print('hello')")

    def test_create_and_delete_file(self):
        # Create
        response = self.client.post(
            "/academy/create_file/",
            {
                "project_id": "file_test_ex",
                "location": "",
                "file_name": "new_file.py",
            },
        )
        self.assertEqual(response.status_code, 200)

        # Delete
        response = self.client.post(
            "/academy/delete_file/",
            {"project_id": "file_test_ex", "path": "new_file.py"},
        )
        self.assertEqual(response.status_code, 200)

    def test_delete_nonexistent_file_returns_404(self):
        response = self.client.post(
            "/academy/delete_file/",
            {"project_id": "file_test_ex", "path": "ghost.py"},
        )
        self.assertEqual(response.status_code, 404)

    def test_upload_binary_file(self):
        content = base64.b64encode(b"\x00\x01\x02\x03").decode("utf-8")
        response = self.client.post(
            "/academy/upload/",
            {
                "project_id": "file_test_ex",
                "file_name": "model.onnx",
                "location": "",
                "content": content,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json()["success"])

    def test_path_traversal_rejected(self):
        response = self.client.post(
            "/academy/create_file/",
            {
                "project_id": "file_test_ex",
                "location": "../",
                "file_name": "evil.py",
            },
        )
        self.assertEqual(response.status_code, 403)


class GetHelperFileViewTests(TestCase):
    """Tests for get_helper_file and get_helper_file_list endpoints."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.fal = FAL_RA(self.tmp, self.tmp)
        from academy import error_handler

        self._orig_fal = error_handler.local_fal
        error_handler.local_fal = FAL_RA(self.tmp, self.tmp)
        Exercise.objects.create(
            exercise_id="helper_test_ex",
            name="Helper Test Exercise",
            description="Test",
            url="helper_test_ex",
        )
        helper_dir = os.path.join(self.tmp, "helper_test_ex", "python_template")
        os.makedirs(helper_dir, exist_ok=True)
        with open(os.path.join(helper_dir, "helper.py"), "w") as f:
            f.write("# helper content")
        with open(os.path.join(helper_dir, "model.onnx"), "wb") as f:
            f.write(b"TESTBIN")

    def tearDown(self):
        from academy import error_handler

        error_handler.local_fal = self._orig_fal
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_get_helper_file_missing_project_returns_400(self):
        response = self.client.get(
            "/academy/get_helper_file/",
            {"language": "python", "filename": "helper.py"},
        )
        self.assertEqual(response.status_code, 400)

    def test_get_helper_file_missing_filename_returns_text_file(self):
        response = self.client.get(
            "/academy/get_helper_file/",
            {
                "project": "helper_test_ex",
                "language": "python",
                "filename": "helper.py",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertIn("content", response.json())

    def test_get_helper_file_binary_returns_base64(self):
        response = self.client.get(
            "/academy/get_helper_file/",
            {
                "project": "helper_test_ex",
                "language": "python",
                "filename": "model.onnx",
                "binary": "true",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertIn("content", response.json())

    def test_get_helper_file_text_without_binary_flag(self):
        response = self.client.get(
            "/academy/get_helper_file/",
            {
                "project": "helper_test_ex",
                "language": "python",
                "filename": "helper.py",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["content"], "# helper content")

    def test_get_helper_file_list_missing_project_returns_400(self):
        response = self.client.get(
            "/academy/get_helper_file_list/",
            {"language": "python"},
        )
        self.assertEqual(response.status_code, 400)

    def test_get_helper_file_list_valid_returns_200(self):
        response = self.client.get(
            "/academy/get_helper_file_list/",
            {"project": "helper_test_ex", "language": "python"},
        )
        self.assertEqual(response.status_code, 200)


class TemplateTests(TestCase):
    """Tests for academy/templates.py select_template function."""

    def test_py_reactive_returns_nonempty(self):
        result = select_template("py-reactive")
        self.assertIsInstance(result, str)
        self.assertGreater(len(result), 0)

    def test_py_ros2_returns_nonempty(self):
        result = select_template("py-ros2")
        self.assertIsInstance(result, str)
        self.assertGreater(len(result), 0)

    def test_cpp_reactive_returns_nonempty(self):
        result = select_template("cpp-reactive")
        self.assertIsInstance(result, str)
        self.assertGreater(len(result), 0)

    def test_cpp_ros2_returns_nonempty(self):
        result = select_template("cpp-ros2")
        self.assertIsInstance(result, str)
        self.assertGreater(len(result), 0)

    def test_unknown_template_returns_empty_string(self):
        self.assertEqual(select_template("unknown"), "")

    def test_empty_string_returns_empty_string(self):
        self.assertEqual(select_template(""), "")

    def test_py_reactive_contains_expected_keywords(self):
        result = select_template("py-reactive")
        self.assertIn("HAL", result)
        self.assertIn("WebGUI", result)
        self.assertIn("Frequency", result)
        self.assertIn("while True", result)

    def test_py_ros2_contains_expected_keywords(self):
        result = select_template("py-ros2")
        self.assertIn("rclpy", result)
        self.assertIn("Node", result)
        self.assertIn("WebGUI", result)

    def test_cpp_reactive_contains_expected_keywords(self):
        result = select_template("cpp-reactive")
        self.assertIn("HAL.hpp", result)
        self.assertIn("WebGUI.hpp", result)
        self.assertIn("Frequency.hpp", result)

    def test_cpp_ros2_contains_expected_keywords(self):
        result = select_template("cpp-ros2")
        self.assertIn("rclcpp", result)
        self.assertIn("Node", result)
