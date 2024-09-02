# Gui Interfaces

Gui interfaces is a python package that provides the hardware abstarction layer for the various sensors and actuators used in the exercises of Robotics Academy.

- [How to use](#How-to-use)
- [How to add interfaces](#How-to-add-interfaces)
- [How to add to an exercise](#How-to-add-to-an-exercise)

<a name="How-to-use"></a>
## How to use

In order to import the specific Gui interface follow the next pattern.

Being for general interfaces:

```python
from gui_interfaces.general.{INTERFACE} import ...
```

where {INTERFACE} needs to be substitued by the interface name.

And for specific interfaces:

```python
from gui_interfaces.specific.{EXERCISE}.{INTERFACE} import ...
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
- Create your exercise GUI that inherits from the chosen GUI interface
- Check in each interface what needs to be implemented.
