# Human Detection Exercise Using Deep Learning

A Human Detection Exercise to identify the presence of humans and identification of the rectangular boundary around them. 
The user is expected to upload a Deep Learning model which fits the required input and output specifications for inference. The input model is supposed to be in the ONNX format. For more information refer to the "Exercise Instructions" section below.

* * *

## Launch Instructions

- There are two ways to run the exercise using web-template:

  - Run the exercise with docker container
   - Run it without container

### Run with docker container

- First you need to build the image. Then, you need to run a container.
```
git clone https://github.com/JdeRobot/RoboticsAcademy.git && cd RoboticsAcademy && git checkout noetic
cd scripts
docker build -f Dockerfile-noetic -t image-name .
docker run -it --name=container_name -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 image-name ./start.sh
```  
- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.
- Click the connect button and wait for some time until an alert appears with the message Connection Established and button displays connected.
- The exercise can be used after the alert.
- It is necessary to map the port where the camera is located to the docker container.

### Run without docker container

The following dependencies should be pre-installed:
- Python 3 or later
- Python dependencies
     - OpenCV
     - onnxruntime
     - WebsocketServer

- Clone the Robotics Academy repository to your local machine, switch to the noetic branch and head over to the Human_Detection exercise.
```
git clone https://github.com/JdeRobot/RoboticsAcademy.git && cd RoboticsAcademy && git checkout noetic
```
      
- Determine your machine dns server IP address which is generally in the form of **127.0.0.xx for Linux machine** by running this command

```bash
cat /etc/resolv.conf
```

- Inside `assets/websocket_address.js` file, change the **variable websocket_address** to the IP address found with the above command

- Start the host application along with the same IP address which is used for connection.

```bash
python exercise.py 127.0.0.xx
```

- Open the browser template from `exercice.html`

- The page should says **[open]Connection established!**.Means it is working as expected.

**__NOTE:__**  If you get **socket.error: [Errno 99] Cannot assign requested address** error,you need to check and pass the correct IP address.

* * *

## Exercise Instructions

- The uploaded ONNX format model should adhere to the input/output specifications, please keep that in mind while building your model. 
- The user can train their model in any framework of their choice and export it to the ONNX format. Refer to this [**article**](https://docs.unity3d.com/Packages/com.unity.barracuda@1.0/manual/Exporting.html) to know more about exporting your model to the ONNX format.

### Model Input Specification

`input_shape` - The application code pre processes the input frame of shape (H, W, C) to (1, 300, 300, C), where 1 represents the batch size . This information is mainly provided for users designing their model with a fully connected input layer.

### Model Output Specification

Given each batch of images, the model must return 4 tensor arrays:

`num_detections`: the number of detections.

`detection_boxes`: a list of bounding boxes. Each list item describes a box with top, left, bottom, right relative to the image size.

`detection_scores`: the score for each detection with values between 0 and 1 representing probability that a class was detected.

`detection_classes`: Array of detected classes. The class label must be **1** for humans. 

Note: Make sure to keep the class label for Humans while training your model as 1. Any object detected by your model with any other class label other than 1, will not be accounted for.

## Demo Model

A demo model has been provided inside the `Demo_Model` folder to test and play around with the application.








