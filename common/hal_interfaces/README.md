# Hal Interfaces

Hal interfaces is a python package that provides the hardware abstarction layer for the various sensors and actuators used in the exercises of Robotics Academy.

- [How to use](#How-to-use)
- [How to add interfaces](#How-to-add-interfaces)
- [How to add to an exercise](#How-to-add-to-an-exercise)

<a name="How-to-use"></a>
## How to use

In order to import the specific HAL interface follow the next pattern.

Being for general interfaces:

```python
from hal_interfaces.general.{INTERFACE} import ...
```

where {INTERFACE} needs to be substitued by the interface name.

And for specific interfaces:

```python
from hal_interfaces.specific.{EXERCISE}.{INTERFACE} import ...
```

where {EXERCISE} is the name of exercise being worked on and {INTERFACE} needs to be substitued by the interface name.

<a name="How-to-add-interfaces"></a>
## How to add interfaces

If the interface is only specific to one exercise add it as a specific interface, if not the add it to general. Also the naming scheme that needs to be followed when naming a new interface file is snake case or lower case with underlines.

For reference, please refer to the existing interfaces.

### General Interfaces

- Inside the general directory

### Specific Interfaces

- Create a directory named after the exercise name inside of the specific directory.

<a name="How-to-add-to-an-exercise"></a>
## How to add to an exercise

- Import each interface as stated in the [How to use](#How-to-use) section.
- Initialize each interface
- For subscribers (interfaces that receive data) it is recommended to add them inside an executor thread as follows:

```python
import rclpy
import sys
import threading
import time

freq = 60.0 # Desired iteration frequency for subscriber callback

def __auto_spin() -> None:
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        time.sleep(1/freq)

# Init the interfaces and start rclpy
#
#####################################

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()

# Add all of the interfaces that need to receive data
executor.add_node(my_interface_1)
executor.add_node(my_interface_2)

executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()
```
