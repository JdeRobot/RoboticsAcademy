# Robotics Application Manager (RAM) Documentation

## Table of Contents

1. [Project Overview](#project-overview)
2. [Main Class: `Manager`](#main-class-manager)
   - [Purpose and Functionality](#purpose-and-functionality)
   - [States and Transitions](#states-and-transitions)
   - [Key Methods](#key-methods)
   - [Interactions with Other Components](#interactions-with-other-components)
3. [Usage Examples](#usage-examples)

## Project Overview

The Robotics Application Manager (RAM) is an advanced manager for executing robotic applications. It operates as a state machine, managing the lifecycle of robotic applications from initialization to termination.

## Main Class: `Manager`

### Purpose and Functionality

The `Manager` class is the core of RAM, orchestrating operations and managing transitions between various application states.

### States and Transitions

- **States**:
  - `idle`: The initial state, waiting for a connection.
  - `connected`: Connected and ready to initiate processes.
  - `world_ready`: The world environment is set up and ready.
  - `visualization_ready`: Visualization tools are prepared and ready.
  - `application_running`: A robotic application is actively running.
  - `paused`: The application is paused.
- **Transitions**:
  - `connect`: Moves from `idle` to `connected`.
  - `launch_world`: Initiates the world setup from `connected`.
  - `prepare_visualization`: Prepares the visualization tools in `world_ready`.
  - `run_application`: Starts the application in `visualization_ready` or `paused`.
  - `pause`: Pauses the running application.
  - `resume`: Resumes a paused application.
  - `terminate`: Stops the application and goes back to `visualization_ready`.
  - `stop`: Completely stops the application.
  - `disconnect`: Disconnects from the current session and returns to `idle`.

### Key Methods

- `on_connect(self, event)`: Manages the transition to the 'connected' state.
- `on_launch_world(self, event)`: Prepares and launches the robotic world.
- `on_prepare_visualization(self, event)`: Sets up visualization tools.
- `on_run_application(self, event)`: Executes the robotic application.
- `on_pause(self, msg)`: Pauses the running application.
- `on_resume(self, msg)`: Resumes the paused application.
- `on_terminate(self, event)`: Terminates the running application.
- `on_disconnect(self, event)`: Handles disconnection and cleanup.
- **Exception Handling**: Details how specific errors are managed in each method.

### Interactions with Other Components

#### Interaction Between `Manager` and `ManagerConsumer`

1. **Message Queue Integration**: `ManagerConsumer` puts received messages into `manager_queue` for `Manager` to process.
2. **State Updates and Commands**: `Manager` sends state updates or commands to the client through `ManagerConsumer`.
3. **Client Connection Handling**: `Manager` relies on `ManagerConsumer` for client connection and disconnection handling.
4. **Error Handling**: `ManagerConsumer` communicates exceptions back to the client and `Manager`.
5. **Lifecycle Management**: `Manager` controls the start and stop of the `ManagerConsumer` WebSocket server.

#### Interaction Between `Manager` and `LauncherWorld`

1. **World Initialization and Launching**: `Manager` initializes `LauncherWorld` with specific configurations, such as world type (e.g., `gazebo`, `drones`) and the launch file path.
2. **Dynamic Module Management**: `LauncherWorld` dynamically launches modules based on the world configuration and ROS version, as dictated by `Manager`.
3. **State Management and Transition**: The state of `Manager` is updated in response to the actions performed by `LauncherWorld`. For example, once the world is ready, `Manager` may transition to the `world_ready` state.
4. **Termination and Cleanup**: `Manager` can instruct `LauncherWorld` to terminate the world environment through its `terminate` method. `LauncherWorld` ensures a clean and orderly shutdown of all modules and resources involved in the world setup.
5. **Error Handling and Logging**: `Manager` handles exceptions and errors that may arise during the world setup or termination processes, ensuring robust operation.

#### Interaction Between `Manager` and `LauncherVisualization`

1. **Visualization Setup**: `Manager` initializes `LauncherVisualization` with a specific visualization configuration, which can include types like `console`, `gazebo_gra`, `gazebo_rae`, etc.
2. **Module Launching for Visualization**: `LauncherVisualization` dynamically launches visualization modules based on the configuration provided by `Manager`.
3. **State Management and Synchronization**: Upon successful setup of the visualization tools, `Manager` can update its state (e.g., to `visualization_ready`) to reflect the readiness of the visualization environment.
4. **Termination of Visualization Tools**: `Manager` can instruct `LauncherVisualization` to terminate the current visualization setup using its `terminate` method.
5. **Error Handling and Logging**: `Manager` is equipped to manage exceptions and errors that might occur during the setup or termination of visualization tools.

#### Interaction Between `Manager` and `application_process`

1. **Application Execution**: `Manager` initiates the `application_process` when transitioning to the `application_running` state.
2. **Application Configuration and Launching**: Before launching the `application_process`, `Manager` configures the necessary parameters.
3. **Process Management**: `Manager` monitors and controls the `application_process`.
4. **Error Handling and Logging**: `Manager` is responsible for handling any errors or exceptions that occur during the execution of the `application_process`.
5. **State Synchronization**: The state of the `application_process` is closely synchronized with the state machine in `Manager`.

#### Interaction Between `Manager` and `Server` (Specific to RoboticsAcademy Applications)

1. **Dedicated WebSocket Server for GUI Updates**: `Server` is used exclusively for RoboticsAcademy applications that require real-time interaction with a web-based GUI.
2. **Client Communication for GUI Module**: For RoboticsAcademy applications with a GUI module, `Server` handles incoming and outgoing messages.
3. **Real-time Interaction and Feedback**: `Server` allows for real-time feedback and interaction within the browser-based GUI.
4. **Conditional Operation Based on Application Type**: `Manager` initializes and controls `Server` based on the specific needs of the RoboticsAcademy application being executed.
5. **Error Handling and Logging**: `Manager` ensures robust error handling for `Server`.

## Usage Example

1. **Connecting to RAM**:

   - Initially, the RAM is in the `idle` state.
   - A client (e.g., a user interface or another system) connects to RAM, triggering the `connect` transition and moving RAM to the `connected` state.

2. **Launching the World**:

   - Once connected, the client can request RAM to launch a robotic world by sending a `launch_world` command.
   - RAM transitions to the `world_ready` state after successfully setting up the world environment.

3. **Setting Up Visualization**:

   - After the world is ready, the client requests RAM to prepare the visualization tools with a `prepare_visualization` command.
   - RAM transitions to the `visualization_ready` state, indicating that visualization tools are set up and ready.

4. **Running an Application**:

   - The client then requests RAM to run a specific robotic application, moving RAM into the `application_running` state.
   - The application executes, and RAM handles its process management, including monitoring and error handling.

5. **Pausing and Resuming Application**:

   - The client can send `pause` and `resume` commands to RAM to control the application's execution.
   - RAM transitions to the `paused` state when paused and returns to `application_running` upon resumption.

6. **Stopping the Application**:

   - Finally, the client can send a `stop` command to halt the application.
   - RAM stops the application and transitions back to the `visualization_ready` state, ready for new commands.

7. **Disconnecting**:
   - Once all tasks are completed, the client can disconnect from RAM, which then returns to the `idle` state, ready for a new session.
