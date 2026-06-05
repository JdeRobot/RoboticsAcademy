# Robotics Academy Tests

This directory contains tests for the Robotics Academy project. The tests are organized into various subdirectories based on their purpose and functionality.

## Directory Structure

```bash
├── conftest.py # Configuration file for pytest
├── console_interfaces # Tests for console interfaces
├── gui_interfaces # Tests for GUI interfaces
├── hal_interfaces # Tests for hardware abstraction layer interfaces
├── __init__.py
├── mocks # Mock objects for testing
└── README.md
```

## Running Tests

-   Run the RADI as stated in the [Developer Docs](../docs/InstructionsForDevelopers.md).

-   Open a terminal in the Docker container
    ```bash
    docker exec -it developer-container bash
    ```
-   Navigate to the RoboticsAcademy directory
    ```bash
    cd RoboticsAcademy/
    ```
-   Install testing dependencies
    ```bash
    pip install -r tests/requirements.txt
    ```
-   Run all tests using pytest
    ```bash
    pytest
    ```
-   To run a specific module tests, use `pytest tests/<module_name>/`. For example, to run GUI interface tests:
    ```bash
    pytest tests/gui_interfaces/
    ```
