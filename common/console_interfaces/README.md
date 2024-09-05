# Console Interfaces

Console interfaces is a python package that provides functions that are used in the exercises of Robotics Academy to manage the console.

- [How to use](#How-to-use)
- [How to add interfaces](#How-to-add-interfaces)
- [How to add to an exercise](#How-to-add-to-an-exercise)

<a name="How-to-use"></a>
## How to use

In order to import the specific Gui interface follow the next pattern.

Being for general interfaces:

```python
from console_interfaces.general.{INTERFACE} import ...
```

where {INTERFACE} needs to be substitued by the interface name.

And for specific interfaces:

```python
from console_interfaces.specific.{EXERCISE}.{INTERFACE} import ...
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
- At the end of th exercise's GUI.py file call the start_console function.
