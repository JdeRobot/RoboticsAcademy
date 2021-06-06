#!/usr/bin/env python

from __future__ import print_function

import base64
from datetime import datetime
import json
import os
import sys
import threading
import time

import cv2

import numpy as np
import onnxruntime as rt
from websocket_server import WebsocketServer

from gui import GUI, ThreadGUI
from hal import HAL
import console


class Template:
    # Initialize class variables
    # self.time_cycle to run an execution for at least 1 second
    # self.process for the current running process
    def __init__(self):
        self.thread = None
        self.reload = False

        # Time variables
        self.time_cycle = 80
        self.ideal_cycle = 80
        self.iteration_counter = 0
        self.frequency_message = {'brain': '', 'gui': ''}

        self.server = None
        self.client = None
        self.host = sys.argv[1]

        self.aux_model_fname = "dummy.onxx"  # internal name for temporary model uploaded by user

        # Initialize the GUI, WEBRTC and Console behind the scenes
        self.console = console.Console()
        self.hal = HAL()
        self.gui = GUI(self.host, self.console, self.hal)

    def output_detection(self, img, d, score):
        """Draw box and label for 1 detection."""
        height, width = img.shape[0], img.shape[1]
        # the box is relative to the image size so we multiply with height and width to get pixels.
        top = d[0] * height
        left = d[1] * width
        bottom = d[2] * height
        right = d[3] * width
        top = max(0, np.floor(top + 0.5).astype('int32'))
        left = max(0, np.floor(left + 0.5).astype('int32'))
        bottom = min(height, np.floor(bottom + 0.5).astype('int32'))
        right = min(width, np.floor(right + 0.5).astype('int32'))
        cv2.rectangle(img, (left, top), (right, bottom), (0,0,255), 2)
        cv2.putText(img, 'Human', (left, top-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (36,255,12), 1)
        cv2.putText(img, str(score)+"%", (left+150, top-10), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255,0,0), 1)
        self.gui.showResult(img, str(""))


    # The process function
    def process_dl_model(self, raw_dl_model):
        """
        Given a DL model in onnx format, yield prediction per frame.
        :param raw_dl_model: raw DL model transferred through websocket
        """
        # Receive model
        raw_dl_model = raw_dl_model.split(",")[-1]
        raw_dl_model_bytes = raw_dl_model.encode('ascii')
        raw_dl_model_bytes = base64.b64decode(raw_dl_model_bytes)

        # Load ONNX model
        try:
            with open(self.aux_model_fname, "wb") as f:
                f.write(raw_dl_model_bytes)
            sess = rt.InferenceSession(self.aux_model_fname)
            # input layer name in the model
            input_layer_name = sess.get_inputs()[0].name
            # list for storing names of output layers of the model
            output_layers_names = []
            for i in range( len(sess.get_outputs()) ):
                output_layers_names.append(sess.get_outputs()[i].name)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            self.console.print(str(exc_value))
            self.console.print("ERROR: Model couldn't be loaded")

        try:
            while not self.reload:
                start_time = datetime.now()
                start = time.time()

                # Get input webcam image
                img = self.hal.getImage()
                img_resized = cv2.resize(img, (300,300))
                img_data = np.reshape(img_resized, (1, img_resized.shape[0], img_resized.shape[1], img_resized.shape[2]))

                # Inference
                # We will get output in the order of output_layers_names
                result = sess.run(output_layers_names, {input_layer_name: img_data})
                detection_boxes, detection_classes, detection_scores, num_detections = result
                count = 0

                batch_size = num_detections.shape[0]
                for batch in range(0, batch_size):
                    for detection in range(0, int(num_detections[batch])):
                        c = detection_classes[batch][detection]
                        # Skip if not human class
                        if c != 1:
                            self.gui.showResult(img, str(count))
                            continue
                        count = count + 1
                        d = detection_boxes[batch][detection]
                        score = detection_scores[batch][detection]
                        self.output_detection(img, d, round(score*100))
                    # To print FPS after each frame as been fully processed and displayed
                    end = time.time()
                    frame_time = round(end-start, 3)
                    fps = 1.0/frame_time
                    cv2.putText(img, "FPS: {}".format(int(fps)), (7,25), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,255,255), 1)
                    self.gui.showResult(img, str(count))

                # Template specifics to run!
                finish_time = datetime.now()
                dt = finish_time - start_time
                ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

                self.iteration_counter += 1

                # The code should be run for atleast the target time step
                # If it's less put to sleep
                if (ms < self.time_cycle):
                    time.sleep((self.time_cycle - ms) / 1000.0)

            print("Process thread closed!")

        # To print the errors that the user submitted through the Javascript editor (ACE)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            self.console.print(str(exc_value))

    # Function to measure the frequency of iterations
    def measure_frequency(self):
        previous_time = datetime.now()
        # An infinite loop
        while self.reload == False:
            # Sleep for 2 seconds
            time.sleep(2)

            # Measure the current time and subtract from the previous time to get real time interval
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time

            # Get the time period
            try:
                # Division by zero
                self.ideal_cycle = ms / self.iteration_counter
            except:
                self.ideal_cycle = 0

            # Reset the counter
            self.iteration_counter = 0
        print("Frequency Thread Closed!")

    # Function to generate and send frequency messages
    def send_frequency_message(self):
        # This function generates and sends frequency measures of the brain and gui
        brain_frequency = 0
        gui_frequency = 0
        try:
            brain_frequency = round(1000 / self.ideal_cycle, 1)
        except ZeroDivisionError:
            brain_frequency = 0

        try:
            gui_frequency = round(1000 / self.thread_gui.ideal_cycle, 1)
        except ZeroDivisionError:
            gui_frequency = 0

        self.frequency_message["brain"] = brain_frequency
        self.frequency_message["gui"] = gui_frequency

        message = "#freq" + json.dumps(self.frequency_message)
        self.server.send_message(self.client, message)

    # Function to maintain thread execution
    def execute_thread(self, dl_model_raw):
        # Keep checking until the thread is alive
        # The thread will die when the coming iteration reads the flag
        if (self.thread != None):
            while self.thread.is_alive() or self.measure_thread.is_alive():
                pass

        # Turn the flag down, the iteration has successfully stopped!
        self.reload = False
        # New thread execution
        self.measure_thread = threading.Thread(target=self.measure_frequency)
        self.thread = threading.Thread(target=self.process_dl_model, args=[dl_model_raw])
        self.thread.start()
        self.measure_thread.start()
        print("Process Thread Started!")
        print("Frequency Thread started!")

    # Function to read and set frequency from incoming message
    def read_frequency_message(self, message):
        frequency_message = json.loads(message)

        # Set brain frequency
        frequency = float(frequency_message["brain"])
        self.time_cycle = 1000.0 / frequency

        # Set gui frequency
        frequency = float(frequency_message["gui"])
        self.thread_gui.time_cycle = 1000.0 / frequency

        return  

    # The websocket function
    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):
        if (message[:5] == "#freq"):
            frequency_message = message[5:]
            self.read_frequency_message(frequency_message)
            self.send_frequency_message()
            return
        elif (message[:5] == "#stop"):
            self.reload = True
            return
        try:
            # Once received turn the reload flag up and send it to execute_thread function
            dl_model_raw = message
            # print(repr(code))
            self.reload = True
            self.execute_thread(dl_model_raw)
        except:
            pass

    # Function that gets called when the server is connected
    def connected(self, client, server):
        self.client = client
        # Start the GUI update thread
        self.thread_gui = ThreadGUI(self.gui)
        self.thread_gui.start()

        # Initialize the ping message
        self.send_frequency_message()

        print(client, 'connected')

    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        if os.path.isfile(self.aux_model_fname):
            os.remove(self.aux_model_fname)  # remove temporary model file when closing
        print(client, 'closed')

    def run_server(self):
        self.server = WebsocketServer(port=1905, host=self.host)
        self.server.set_fn_new_client(self.connected)
        self.server.set_fn_client_left(self.handle_close)
        self.server.set_fn_message_received(self.handle)
        self.server.run_forever()


# Execute!
if __name__ == "__main__":
    server = Template()
    server.run_server()
