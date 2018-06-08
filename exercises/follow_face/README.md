                        FOLLOW FACE EXCERSISE
                        =====================

In this practice, the intention is to use your knowledge in image processing to 
segment the faces of people and follow them through a camera connected by USB to 
your computer. For this, you must have the right hardware (Sony model EVI d100p),
and then implement the logic that performs the segmentation of a face and an 
algorithm that collects that data and transforms it into orders for the camera's 
actuators, which must follow the movement of the person.

////////////////////////////////////////////////////////////////////////////////
                           E X E C U T I O N 
////////////////////////////////////////////////////////////////////////////////

* Note that to make this component run, you need a image provider (a camera) 
connected to an USB port and listening at the ports you want to bind, namely, 
properly configured (through .cfg and .yml files). So first you have to run both 
usb cam and evi cam and then run the follow_face component.

The way to make it run is the following:

1. Run "usb_cam" in the first terminal(ensure you have installed ros-kinetic-usb-cam packet):

`$ roslaunch usb_cam-test.launch`

2. Run the evicam_driver in the second:

`$ evicam_driver evicam_driver.cfg` **

3. Finally, Run the follow_face component:

`$ python2 ./follow_face.py follow_face_conf.yml`

* To simplify the closure of the environment, just close the Autopark window (s). 
  Ctrl + C will give problems.
** If you obtain the following output in step 2
    [Info] loaded Ice.Config file: evicam_driver.cfg
    Errro EVILib Open(int, char *) 9 EBADF
    Error cam Open
    !! 12/14/17 11:56:54.213 error: communicator not destroyed during global destruc
try: `$ sudo chmod 777 /dev/ttyUSB0` and retry step 2.
////////////////////////////////////////////////////////////////////////////////


# Configuration files

If your system has a local camera, make sure that in the file "usb_cam-test.lauch" 
you put the correct path to the camera connected by usb (/dev/video1). Otherwise, 
put the default route (/dev/video0). This is:

[usb_cam-test.lauch](usb_cam-test.lauch#L3)
```
            <param name="video_device" value="/dev/video0" />
                                or
            <param name="video_device" value="/dev/video1" />
```

## How to do the practice
To carry out the practice, you must edit the MyAlgorithm.py file and insert 
the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L65)
```
        def execute(self):
            # Add your code here
            # Input image
            input_image = self.camera.getImage()
            if input_image is not None:
                self.camera.setColorImage(input_image)
                '''
                If you want show a thresohld image (black and white image) or a gray 
                or filtered image use:
                self.camera.setThresholdImage(image)
                '''
        
```

### API
* self.camera.getImage(): returns the image captured by the active camera (local or usb_cam).
* self.camera.setColorImage(img): to show a color image.
* self.camera.setThresholdImage(img): If you want show a black and white or a gray or filtered image.
* self.motors.getLimits(): obtain pan and tilt limits (in degrees).
* self.motors.setPTMotorsData(pan, tilt, panSpeed, tiltSpeed): send pan and tilt values to the camera.


## Demonstrative video
https://www.youtube.com/watch?v=pdSPnftumf8

* *Drivers and basic GUI by Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
* *Academic node, extended GUI and solution by Carlos Awadallah <carlosawadallah@gmail.com>


