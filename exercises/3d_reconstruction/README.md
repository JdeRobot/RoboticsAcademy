# 3D RECONSTRUCTION EXCERSISE
                        
In this practice, the intention is to program the necessary logic to allow kobuki
robot to generate a 3D reconstruction of the scene that it is receiving throughout its
left and right cameras.



## PREPARATION
Follow these simple steps:

1. copy the new interface:

```
sudo cp interface/visualization_ice.py /opt/jderobot/lib/python2.7
```
2.Prepare 3d viewer:

```
sudo apt-get install nodejs
sudo apt-get install npm
cd 3DVizWeb
npm install electron --save-dev --save-exact
npm install jquery
npm install js-yaml
```

## EXECUTION


Follow these simple steps to launch the practice:

1. First of all, run Gazebo simulator:
    * Execution without seeing the world:
        `$ gzserver reconstruccion3D.world`
    * Normal execution (seeing the world):
        `$ gazebo reconstruccion3D.world`
        
2. Then, run the 3d_reconstruction component:
    `$ python2 3d_reconstruction.py 3d_reconstruction_conf.yml`

3. Finally, launch 3D viewer tool:
    ```
    cd 3DVizWeb
    npm start
    ```

* To simplify the closure of the environment, just close the Autopark window (s).
  `Ctrl + C` will give problems.

* To change the configuration of `3DVizWeb`:
    ```
    Open 3DVizWeb/public/config.yml
    Modify only the next fields (updatePoints, updateSegments, linewidth, pointsize, camera)
    ```

---------


## How to do the practice
To carry out the practice, you must edit the `MyAlgorithm.py` file and insert
the control logic into it.

### Where to insert the code?
[MyAlgorithm.py](MyAlgorithm.py#L41)
```
    def execute(self):

        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()

        if self.done:
            return

        # Add your code here
        # pointIn=np.array([502,21,1])
        # pointInOpt=self.camLeftP.graficToOptical(pointIn)
        # point3d=self.camLeftP.backproject(pointInOpt)
        # projected1 = self.camRightP.project(point3d)
        # print (self.camRightP.opticalToGrafic(projected1))
        . . .
```

### API
* `motors.setV()` - to set linear velocity
* `motors.setW()` - set angular velocity
* `motors.sendVelocities()` - to send the velocities set.
* `motors.sendV()` - to send linear velocity only
* `motors.sendW()` - send angular velocity only
* `self.setLeftImageFiltered(imageRight)` - shows filtered image on the left position of the GUI
* `self.setRightImageFiltered(imageRight)` - shows filtered image on the right position of the GUI


* PLOT 3D data on the viewer:
   ```
   point=np.array([1, 1, 1])
   color=(255,255,255)
   self.sensor.drawPoint(point,color[1],color[2],color[3]))
   ```

## Demonstrative video
