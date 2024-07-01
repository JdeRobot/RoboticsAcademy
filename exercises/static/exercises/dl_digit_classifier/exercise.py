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
import onnxruntime
from onnxruntime.quantization import quantize_dynamic, QuantType
from websocket_server import WebsocketServer

from gui import GUI, ThreadGUI
from hal import HAL
from console import start_console, close_console


class Template:
    # Initialize class variables
    # self.ideal_cycle to run an execution for at least 1 second
    # self.process for the current running process
    def __init__(self):
        self.thread = None
        self.reload = False

        # Time variables
        self.ideal_cycle = 80
        self.measured_cycle = 80
        self.iteration_counter = 0
        self.frequency_message = {'brain': '', 'gui': ''}

        self.server = None
        self.client = None
        self.host = sys.argv[1]

        self.aux_model_fname = "dummy.onxx"  # internal name for temporary model uploaded by user

        # Initialize the GUI, WEBRTC and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI(self.host, self.hal)

    # The process function
    # The process function
	def process_dl_model(self, raw_dl_model, roi_scale=0.75, input_size=(28, 28)):
        """
        Given a DL model in onnx format, yield prediction per frame.
        :param raw_dl_model: raw DL model transferred through websocket
        :param roi_scale: float, pct of the smallest image dimension that will be used to build the ROI used as input
        :param input_size: (int, int), model input image size
        """
        # Redirect the information to console
        start_console()

        # Receive model
        raw_dl_model = raw_dl_model.split(",")[-1]
        raw_dl_model_bytes = raw_dl_model.encode('ascii')
        raw_dl_model_bytes = base64.b64decode(raw_dl_model_bytes)

        # Load and optimize ONNX model
        ort_session = None
        try:
            with open(self.aux_model_fname, "wb") as f:
                f.write(raw_dl_model_bytes)

            # Load the original model
            model = onnx.load(self.aux_model_fname)

            # Apply optimizations directly using ONNX Runtime
            model_optimized = onnx.optimizer.optimize(model, passes=[
                "eliminate_identity", 
                "eliminate_deadend",
                "eliminate_nop_dropout", 
                "eliminate_nop_transpose",
                "fuse_bn_into_conv", 
                "fuse_consecutive_transposes",
                "fuse_pad_into_conv", 
                "fuse_transpose_into_gemm",
                "lift_lexical_references",
                "nop_elimination",
                "split_init"
            ])

            # Save the optimized model
            optimized_model_fname = "optimized_model.onnx"
            onnx.save(model_optimized, optimized_model_fname)

            # Quantize the model
            quantized_model_fname = "quantized_model.onnx"
            quantize_dynamic(optimized_model_fname, quantized_model_fname, weight_type=QuantType.QInt8)

            # Load the quantized model
            ort_session = onnxruntime.InferenceSession(quantized_model_fname)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))
            print("ERROR: Model couldn't be loaded or optimized")

        try:
            # Init auxiliar variables used for stabilized predictions
            previous_pred = 0
            previous_established_pred = "-"
            count_same_digit = 0
            while not self.reload:
                start_time = datetime.now()
                start = time.time()

                # Get input webcam image
                input_image = self.hal.getImage()
                input_image_gray = np.mean(input_image, axis=2).astype(np.uint8)

                # Get original image and ROI dimensions
                h_in, w_in = input_image.shape[:2]
                min_dim_in = min(h_in, w_in)
                h_roi, w_roi = (int(min_dim_in * roi_scale), int(min_dim_in * roi_scale))
                h_border, w_border = (int((h_in - h_roi) / 2.), int((w_in - w_roi) / 2.))

                # Extract ROI and convert to tensor format required by the model
                roi = input_image_gray[h_border:h_border + h_roi, w_border:w_border + w_roi]
                roi_norm = (roi - np.mean(roi)) / np.std(roi)
                roi_resized = cv2.resize(roi_norm, input_size)
                input_tensor = roi_resized.reshape((1, 1, input_size[0], input_size[1])).astype(np.float32)

                # Inference
                ort_inputs = {ort_session.get_inputs()[0].name: input_tensor}
                output = ort_session.run(None, ort_inputs)[0]
                pred = int(np.argmax(output, axis=1))  # get the index of the max log-probability

                end = time.time()
                frame_time = round(end - start, 3)
                fps = 1.0 / frame_time
                # number of consecutive frames that must be reached to consider a validprediction
                n_consecutive_frames = int(fps / 2)

                # For stability, only show digit if detected in more than n consecutive frames
                if pred != previous_established_pred:
                    if count_same_digit < n_consecutive_frames:
                        if previous_pred == pred:
                            count_same_digit += 1
                        else:
                            count_same_digit = 0
                        previous_established_pred = "-"  # no prediction
                    else:
                        print("Digit found: {}".format(pred))
                        previous_established_pred = pred
                        count_same_digit = 0
                    previous_pred = pred

                # Show region used as ROI
                cv2.rectangle(input_image, pt2=(w_border, h_border), pt1=(w_border + w_roi, h_border + h_roi), color=(255, 0, 0), thickness=3)
                # Show FPS count
                cv2.putText(input_image, "FPS: {}".format(int(fps)), (7, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Send result
                self.gui.showResult(input_image, str(previous_established_pred))

                # Template specifics to run!
                finish_time = datetime.now()
                dt = finish_time - start_time
                ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

                self.iteration_counter += 1

                # The code should be run for at least the target time step
                # If it's less put to sleep
                if (ms < self.ideal_cycle):
                    time.sleep((self.ideal_cycle - ms) / 1000.0)

            close_console()
            print("Current Thread Joined!")

        # To print the errors that the user submitted through the Javascript editor (ACE)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))


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
                self.measured_cycle = ms / self.iteration_counter
            except:
                self.measured_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

    # Function to generate and send frequency messages
    def send_frequency_message(self):
        # This function generates and sends frequency measures of the brain and gui
        brain_frequency = 0
        gui_frequency = 0
        try:
            brain_frequency = round(1000 / self.measured_cycle, 1)
        except ZeroDivisionError:
            brain_frequency = 0

        try:
            gui_frequency = round(1000 / self.thread_gui.measured_cycle, 1)
        except ZeroDivisionError:
            gui_frequency = 0

        self.frequency_message["brain"] = brain_frequency
        self.frequency_message["gui"] = gui_frequency

        message = "#freq" + json.dumps(self.frequency_message)
        self.server.send_message(self.client, message)

    def send_ping_message(self):
        self.server.send_message(self.client, "#ping")

    # Function to notify the front end that the code was received and sent to execution
    def send_code_message(self):
        self.server.send_message(self.client, "#exec")

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
        self.send_code_message()
        print("New Thread Started!")

    # Function to read and set frequency from incoming message
    def read_frequency_message(self, message):
        frequency_message = json.loads(message)

        # Set brain frequency
        frequency = float(frequency_message["brain"])
        self.ideal_cycle = 1000.0 / frequency

        # Set gui frequency
        frequency = float(frequency_message["gui"])
        self.thread_gui.ideal_cycle = 1000.0 / frequency

        return

    # The websocket function
    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):
        if (message[:5] == "#freq"):
            frequency_message = message[5:]
            self.read_frequency_message(frequency_message)
            self.send_frequency_message()
            return

        elif(message[:5] == "#ping"):
            time.sleep(1)
            self.send_ping_message()
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

        logged = False
        while not logged:
            try:
                f = open("/ws_code.log", "w")
                f.write("websocket_code=ready")
                f.close()
                logged = True
            except:
                time.sleep(0.1)

        self.server.run_forever()


# Execute!
if __name__ == "__main__":
    server = Template()
    server.run_server()
