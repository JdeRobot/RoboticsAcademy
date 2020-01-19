# Kinect recipe
Recipe to install the drivers and use the Kinect V1 with ROS.

## 1) Install Kinect Sensor drivers:
git clone git://github.com/ph4m/SensorKinect.git
cd SensorKinect/Platform/Linux/CreateRedist
sudo chmod +x RedistMaker
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-x64-v*
sudo ./install.sh

## 2) Install OpenNI camera driver:
sudo apt-get install ros-kinetic-openni-camera ros-kinetic-openni-launch

## 3) See which device identifier is associated (in my case 2):
lsusb

## 4) Launch ROS OpenNI specifying the device:
roslaunch openni_launch openni.launch device_id:=#2

## 5) Launch rviz viewer in another terminal:
rosrun rviz rviz

NOTE: Here we have to add ("Add" button) on the left the type "PointCloud2" and specify the ROS topic that pours the points in depth (see which topic has launched roslaunch), in my case it is "/camera/depth_points". Also, we specify the parameter "Color transformation" as "Axis color".

## Credits
* *Julio Vega (@jmvega)*
