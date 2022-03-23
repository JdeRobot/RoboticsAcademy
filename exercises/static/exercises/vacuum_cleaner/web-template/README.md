[Exercise Documentation Website](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner)


## Threads documentation of vacuum_cleaner exercise

**Thread** --> Name of the thread
**Location** --> Function in which we can find the thread 
**Description** --> Brief description of thread's function
**Target** --> The target variable sets the function that each thread will execute
**Arguments** --> Arguments used in the execution of the function 

### Exercise.py 

| Thread                 | Location        | Description                                                                           | Target          | Arguments |
|------------------------|-----------------|---------------------------------------------------------------------------------------|-----------------|-----------|
| `self.measured_thread` | `connected()`   | It measures the frequency of iterations.                               | `self.measure_frequency `      | None  |
| `self.stats_thread`    | `connected()`   | Tracks the real time factor from Gazebo statistics.                           | `self.track_stats`      | None  |
| `self.thread`          | `execute_thread()`| Redirects the information to console and runs sequential and iterative code (takes care of the compute engine) | `self.process_code` | source_code  |

### gui.py

| Thread                 | Location                    | Description                                                               | Target          | Arguments |
|------------------------|-----------------------------|---------------------------------------------------------------------------|-----------------|-----------|
| `t`                    | class GUI `execute_thread()`| It activates the server                                                   | `self.run_server` | None  |
| `self.measured_thread` | class ThreadGUI `start()`   | Measuring thread to measure frequency.                                    | `self.measure_frequency `      | None  |
| `self.thread`          | class ThreadGUI `start()`   | The main thread of execution.                                             | `self.track_stats`      | None  |

