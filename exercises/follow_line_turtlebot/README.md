# Follow_line_turtlebot exercise
The objective of this practice is to perform a PID reactive control capable of following the line painted on the ground.

## How to execute?
To launch the infrastructure of this practice, first launch the simulator with the appropriate scenario:
```
roslaunch /opt/jderobot/share/jderobot/gazebo/launch/turtlebot-xtion-followline.launch
```
Then you have to execute the academic application, which will incorporate your code:
```
python2 follow_line_turtlebot.py follow_line_turtlebot_conf.yml
```

## How to do the practice?
To carry out the practice, you have to edit the file `MyAlgorithm.py` and insert in it your code, which gives intelligence to the autonomous car.

## Where to insert the code?
[MyAlgorithm.py](MyAlgorithm.py#L109)
```
    def algorithm(self): # Add your code here
        #EXAMPLE: GETTING IMAGES
        image = self.getImage()
        image2 = self.get_threshold_image()

        #EXAMPLE: PRINTING MESSAGES ON THE TERMINAL
        #print "Running"

        #EXAMPLE: SENDING COMMANDS TO THE MOTORS
        #self.motors.setV(10)
        #self.motors.setW(5)

        #EXAMPLE: SHOWING IMAGES ON THE GUI
        self.set_color_image(image)
        #self.set_threshold_image (image2)

```

## API
* `self.getImage()` - to get the image 
* `self.motors.setV()` - to set the linear speed
* `self.motors.setW()` - to set the angular velocity
* `self.set_threshold_image()` - allows you to view a debug image or with relevant information. It must be an image in RGB format (Tip: np.dstack())

## Real Follow Line using Turtlebot in Docker

### 1. Prepare Enviroment
#### 1.1 Install Docker

```bash
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
    
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Add user to docker group
```bash
sudo gpasswd -a ${USER} docker
```
Now, **logout or reboot PC**

#### 1.2. Download academy exercises

```bash
git clone https://github.com/JdeRobot/RoboticsAcademy.git
```

#### 1.3. Download Docker Image

This may take a few minutes

```bash
docker pull jderobot/exercises:turtlebot
```
### 2. Running Exercise

#### 2.1. Share x-server

With this command, we share the display between the docker and your local pc.

```bash
xhost +local:docker
```
The console will return the following message:

```bash
non-network local connections being added to access control list
```


#### 2.2. Run Driver

Turn on turtlebot and connect it and xtion to the PC. 

Then, run following to start docker container:

```bash
docker run -ti --name follow_line --rm --privileged -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/ttyUSB0:/dev/kobuki -v /sys/fs/cgroup:/sys/fs/cgroup:ro -v /dev/serial:/dev/serial -v /dev/bus/usb:/dev/bus/usb -v $HOME/RoboticsAcademy:/root/RoboticsAcademy jderobot/exercises:turtlebot
```
```
roslaunch turtlebot-xtion.launch
```
#### 2.3. Run Node

Open another terminal and execute.

```bash
docker exec -ti follow_line bash
cd /root/RoboticsAcademy/exercises/follow_line_turtlebot
python follow_line_turtlebot.py follow_line_turtlebot_conf.yml
```

### 3. Editing code
Use any editor in **your** PC and then restart the node.

## Credits
* *Base code made by Alberto Martín (@almartinflorido)*
* *Code of practice performed by Julio Vega (@jmvega)*
* *Gazebo models and worlds made by Julio Vega (@jmvega)*
