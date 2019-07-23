# AMAZON WAREHOUSE EXERCISE

The goal of this practice is to implement the logic of a pick, navigate and place robot. The robot must generate shortest path to pallet, navigate to it, pick and deliver it to the delivery zone autonomously.

## How to run
To launch the example, follow the steps below:

0. Add following packages if you don't have them already:
```
$ sudo wget https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml -P /opt/ros/kinetic/share/hector_pose_estimation/
$ sudo apt-get install ros-kinetic-navigation
```

1. Source the gazebo setups:

```
$ source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
$ source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

2. Run Gazebo simulator:

```
$ cd launch
$ ROS_HOME=`pwd` roslaunch amazonrobot_1_warehouse.launch 
```

3. Navigate to exercise folder and run the practice and the user interface: 

```
$ python2 amazonWarehouse.py amazonMap.conf amazonConf.yml
```

To simplify the closure of the environment, just close the Amazon window (s). *`Ctrl + C` will give problems.*


## How to do the practice
To carry out the practice, you must edit the `MyAlgorithm.py` file and insert the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L89)
```
    def execute(self):
        # Add your code here
        print("Running")

        # LIFT PALLET
        self.liftDropExecute()

        # TO DO
```

### API
* `self.client.sendGoalToClient(x, y)` - to send goal to the move_base client
* `self.client.getResultFromClient()` - to send get result from client. None if not reached the goal
* `self.grid.getDestiny()` - to get destination of the robot after double click on GUI
* `self.path.getPath()` - to obtain the path to the current goal
* `self.liftDropExecute()` - to lift or drop pallet
* `clearCostmaps()` - to clear costmaps after moving with pallet

For this example, you have to get the robot autonomously navigate in warehouse environment.

## Demonstrative video of completed solution
[Video](https://www.youtube.com/watch?v=SjvxiyvfyUg)

