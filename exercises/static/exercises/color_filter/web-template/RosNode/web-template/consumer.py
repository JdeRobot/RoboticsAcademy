#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import threading
import json
import cv2
import base64
import numpy as np
from websocket_server import WebsocketServer

# WebRTC Frame Class
class RosConsumer:
    # Initialization function
    # The actual initialization
    def __init__(self, host):
        t = threading.Thread(target=self.run_server)
        self.reload = False
        self.server = None
        self.client = None
        self.host = host
        t.start()

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initWeb(cls, host):
        new_instance = cls(host)
        return new_instance

    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):

        if(message == "#pong"):
            self.server.send_message(self.client, "#ping")
            return

        try:
            # Once received turn the reload flag up and send it to execute_thread function
            message_array=np.fromstring(message, dtype=int, sep=',')
            message_array.resize(240,320,3)
            frame_int64 = message_array
            frame_bgr = np.uint8(frame_int64)
            self.frame_rgb = cv2.cvtColor(frame_bgr ,cv2.COLOR_BGR2RGB)
            self.server.send_message(self.client, "#ack")
            self.reload = True
        except:
            pass

    # Function that gets called when the connected closes
    def getImage(self):
        return self.frame_rgb

    def connected(self, client, server):
        self.client = client

        # Initialize the ping message
        self.server.send_message(self.client, "#ping")

        print(client, 'connected')

    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        print(client, 'closed')

    def run_server(self):
        self.server = WebsocketServer(port=60002, host=self.host)
        self.server.set_fn_new_client(self.connected)
        self.server.set_fn_client_left(self.handle_close)
        self.server.set_fn_message_received(self.handle)
        self.server.run_forever()



    def callback(self,data):

      # Used to convert between ROS and OpenCV images
      br = CvBridge()

      # Output debugging information to the terminal
      rospy.loginfo("receiving video frame")

      # Convert ROS Image message to OpenCV image
      current_frame = br.imgmsg_to_cv2(data)

      _, im_arr = cv2.imencode('.jpg', current_frame)  # im_arr: image in Numpy one-dim array format.
      im_bytes = im_arr.tobytes()
      im_b64 = base64.b64encode(im_bytes)

      try:
        self.server.send_message(self.client, im_b64)
      except:
          pass
    def receive_message(self):

      # Tells rospy the name of the node.
      # Anonymous = True makes sure the node has a unique name. Random
      # numbers are added to the end of the name.
      rospy.init_node('video_sub_py', anonymous=True)

      # Node is subscribing to the video_frames topic
      rospy.Subscriber('video_frames', Image, self.callback)

      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()


if __name__ == '__main__':
  ros =  RosConsumer("0.0.0.0")
  ros.receive_message()
