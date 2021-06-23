# Deep Learning based Human Detection Exercise using WebTemplate

## How to launch the exercise?

- There are two ways to run the exercise using web-template:

      - Run the exercise with docker container
      - Run it without container

## Run with docker container

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

## Run without docker container

- Python 3 or later
- Python dependencies
      - OpenCV
      - onnxruntime
      - WebsocketServer
      
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
