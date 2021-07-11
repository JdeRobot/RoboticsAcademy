#!/usr/bin/env python

from __future__ import print_function

import base64
from datetime import datetime
import json
import os
import sys
import threading
import time
import netron
import shutil
import cv2

import numpy as np
import onnxruntime as rt
from websocket_server import WebsocketServer

from gui import GUI, ThreadGUI
from hal import HAL
from console import start_console, close_console

import _init_paths #Set up path for the benchamrking folder 
# importing required dependencies from benchmarking folder     
from Evaluator import *
import benchmark 



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

        self.aux_model_fname = "dummy.onnx"  # internal name for temporary model uploaded by user

        # Initialize the GUI, WEBRTC and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI(self.host, self.hal)
        

    def video_infer(self):
        # Load ONNX model
        try:
            sess = rt.InferenceSession(self.aux_model_fname)
            # input layer name in the model
            input_layer_name = sess.get_inputs()[0].name
            # list for storing names of output layers of the model
            output_layers_names = []
            for i in range( len(sess.get_outputs()) ):
                output_layers_names.append(sess.get_outputs()[i].name)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))
            print("ERROR: Model couldn't be loaded")
        try:
            while not self.reload:
                start_time = datetime.now()
                start = time.time()
                # Get the uploaded video frame by frame
                success, img = self.hal.getVid()
                if not success:
                    break
                img_resized = cv2.resize(img, (300,300))
                img_data = np.reshape(img_resized, (1, img_resized.shape[0], img_resized.shape[1], img_resized.shape[2]))

                # Inference
                # We will get output in the order of output_layers_names
                result = sess.run(output_layers_names, {input_layer_name: img_data})
                detection_boxes, detection_classes, detection_scores, num_detections = result
                count = 0
                detections = []
                scores = []
                # height, width = img.shape[0], img.shape[1]
                batch_size = num_detections.shape[0]
                for batch in range(0, batch_size):
                    for detection in range(0, int(num_detections[batch])):
                        c = detection_classes[batch][detection]
                        # Skip if not human class
                        if c != 1:
                            self.gui.showResult(img, str(" "))
                            continue
                        count = count + 1
                        #d = detection_boxes[batch][detection]
                        d = detection_boxes[batch][detection]
                        detections.append(d)
                        score = detection_scores[batch][detection]
                        scores.append(score)
                    self.display_output_detection(img, detections, scores)
                    self.hal.frame_number += 1
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

                # The code should be run for at least the target time step
                # If it's less put to sleep
                if (ms < self.time_cycle):
                    time.sleep((self.time_cycle - ms) / 1000.0)

            self.hal.frame_number = 0
            print("Process thread closed!")
        # To print the errors that the user submitted through the Javascript editor (ACE)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))

    
    def saveModel(self, raw_dl_model):
        # Receive model
        raw_dl_model = raw_dl_model.split(",")[-1]
        raw_dl_model_bytes = raw_dl_model.encode('ascii')
        raw_dl_model_bytes = base64.b64decode(raw_dl_model_bytes)
        try:
            with open(self.aux_model_fname, "wb") as f:
                f.write(raw_dl_model_bytes)
        except:
            print("Error saving model to file")


    def saveVideo(self, raw_video):
        print("Received raw video")
        try:
            raw_video = raw_video.split(",")[-1]
            raw_video_bytes = raw_video.encode('ascii')
            raw_video_bytes = base64.b64decode(raw_video_bytes)
            with open("uploaded_video.mp4", "wb") as f:
                f.write(raw_video_bytes)
                print("Video Saved")
        except:
            print("Error in decoding")

    def display_output_detection(self, img, detections, scores):
        """Draw box and label for the detections."""
        # The output detections received from the model are in form [ymin, xmin, ymax, xmax]
        height, width = img.shape[0], img.shape[1]
        for i,detection in enumerate(detections):
            # the box is relative to the image size so we multiply with height and width to get pixels.
            top = detection[0] * height
            left = detection[1] * width
            bottom = detection[2] * height
            right = detection[3] * width
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(height, np.floor(bottom + 0.5).astype('int32'))
            right = min(width, np.floor(right + 0.5).astype('int32'))
            cv2.rectangle(img, (left, top), (right, bottom), (0,0,255), 2)
            cv2.putText(img, 'Human', (left, top-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 1)
            #cv2.putText(img, str(scores[i])+"%", (left+150, top-10), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255,0,0), 1)


    def display_gt_detection(self, img, gt_detections):
        # The ground truth detections received are in the format [xmin, ymin, xmax, ymax]
        for gt_detection in gt_detections:
            left = int(gt_detection[1])
            top = int(gt_detection[2])
            right = int(gt_detection[3])
            bottom = int(gt_detection[4])
            cv2.rectangle(img, (left, top), (right, bottom), (0,255,0), 2)


    def visualizeModel(self):
        try:
            netron.start(self.aux_model_fname)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))
            print("ERROR: Model couldn't be loaded to the visualizer")

        
    # The process function
    def process_dl_model(self):
        """
        Given a DL model in onnx format, yield prediction per frame.
        """

        # Load ONNX model
        try:
            sess = rt.InferenceSession(self.aux_model_fname)
            # input layer name in the model
            input_layer_name = sess.get_inputs()[0].name
            # list for storing names of output layers of the model
            output_layers_names = []
            for i in range( len(sess.get_outputs()) ):
                output_layers_names.append(sess.get_outputs()[i].name)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))
            print("ERROR: Model couldn't be loaded")

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
                detections = []
                scores = []

                batch_size = num_detections.shape[0]
                for batch in range(0, batch_size):
                    for detection in range(0, int(num_detections[batch])):
                        c = detection_classes[batch][detection]
                        # Skip if not human class
                        if c != 1:
                            self.gui.showResult(img, str(count))
                            continue
                        count = count + 1
                        #d = detection_boxes[batch][detection]
                        detections.append(detection_boxes[batch][detection])
                        #score = detection_scores[batch][detection]
                        scores.append(detection_scores[batch][detection])
                    self.display_output_detection(img, detections, scores)
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
            print(str(exc_value))


    def perform_benchmark(self):
        #netron.start("Test_Model/Demo_Model.onnx")
        currentPath = os.path.dirname(os.path.abspath(__file__))
        acc_AP = 0
        validClasses = 0
        # Read txt files containing bounding boxes (ground truth and detections)
        # getBoundingBoxes() function is present inside benchmark.py
        boundingboxes = benchmark.getBoundingBoxes()
        # getBoundingBoxes() changes the current directory, so we need to change it back
        os.chdir(currentPath)
        savePath = os.path.join(currentPath, 'benchmarking/results')
        shutil.rmtree(savePath, ignore_errors=True)
        os.makedirs(savePath)
        # Create an evaluator object in order to obtain the metrics
        evaluator = Evaluator()
        # Plot Precision x Recall curve
        detections = evaluator.PlotPrecisionRecallCurve(
            boundingboxes,  # Object containing all bounding boxes (ground truths and detections)
            IOUThreshold=0.3,  # IOU threshold
            method=MethodAveragePrecision.EveryPointInterpolation,  # As the official matlab code
            showAP=True,  # Show Average Precision in the title of the plot
            showInterpolatedPrecision= True, # Plot the interpolated precision curve
            savePath = savePath, showGraphic=False)
        plot_img  = cv2.imread(os.path.join(savePath, "person.png"))
        self.gui.showResult(plot_img, "Plot")
        f = open(os.path.join(savePath, 'results.txt'), 'w')
        f.write('Average Precision (AP), Precision and Recall: ')

        # each detection is a class
        for metricsPerClass in detections:
            # Get metric values per each class
            cl = metricsPerClass['class']
            ap = metricsPerClass['AP']
            precision = metricsPerClass['precision']
            recall = metricsPerClass['recall']
            totalPositives = metricsPerClass['total positives']
            total_TP = metricsPerClass['total TP']
            total_FP = metricsPerClass['total FP']

            if totalPositives > 0:
                validClasses = validClasses + 1
                acc_AP = acc_AP + ap
                prec = ['%.2f' % p for p in precision]
                rec = ['%.2f' % r for r in recall]
                ap_str = "{0:.2f}%".format(ap * 100)
                print('AP: %s (%s)' % (ap_str, cl))
                f.write('\n\nClass: %s' % cl)
                f.write('\nAP: %s' % ap_str)
                f.write('\n\nPrecision: %s' % prec)
                f.write('\n\n')
                f.write("*" * 100)
                f.write('\nRecall: %s' % rec)
                f.write('\n')
                f.write("*" * 100)

        mAP = acc_AP / validClasses
        mAP_str = "{0:.2f}%".format(mAP * 100)
        print('mAP: %s' % mAP_str)
        f.write('\nmAP: %s' % mAP_str)
        print(f)
        f.close()


    def eval_dl_model(self):
        # Load ONNX model
        try:
            sess = rt.InferenceSession(self.aux_model_fname)
            # input layer name in the model
            input_layer_name = sess.get_inputs()[0].name
            # list for storing names of output layers of the model
            output_layers_names = []
            for i in range( len(sess.get_outputs()) ):
                output_layers_names.append(sess.get_outputs()[i].name)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print(str(exc_value))
            print("ERROR: Model couldn't be loaded")
        try:
            while not self.reload:
                start_time = datetime.now()
                start = time.time()
                # Get the benchmark video frame by frame, and the corresponding ground truth detection values for the frame
                success, img, gt_detections, frame_number = self.hal.getBenchmarkVid()
                if not success:
                    break
                #img_resized = cv2.resize(img, (300,300))
                img_data = np.reshape(img, (1, img.shape[0], img.shape[1], img.shape[2]))

                # Inference
                # We will get output in the order of output_layers_names
                result = sess.run(output_layers_names, {input_layer_name: img_data})
                detection_boxes, detection_classes, detection_scores, num_detections = result
                #count = 0
                detections = []
                scores = []
                height, width = img.shape[0], img.shape[1]
                f = open("benchmarking/detections/" + str(frame_number) + ".txt", "w")
                batch_size = num_detections.shape[0]
                for batch in range(0, batch_size):
                    for detection in range(0, int(num_detections[batch])):
                        c = detection_classes[batch][detection]
                        # Skip if not human class
                        if c != 1:
                            self.gui.showResult(img, str(" "))
                            continue
                        #count = count + 1
                        #d = detection_boxes[batch][detection]
                        d = detection_boxes[batch][detection]
                        detections.append(d)
                        score = detection_scores[batch][detection]
                        scores.append(score)
                        f.write("person" + " ")
                        f.write(str(score) + " ")
                        # The model outputs in the form [ymin, xmin, ymax, xmax]
                        # But out benchmarking code requires the detections to be stored in the format [xmin, ymin, xmax, ymax], so we write to the file accordingly
                        f.write(str(int(d[1]*width)) + " ")
                        f.write(str(int(d[0]*height)) + " ")
                        f.write(str(int(d[3]*width)) + " ")
                        f.write(str(int(d[2]*height)) + "\n")
                    self.display_output_detection(img, detections, scores)
                    self.display_gt_detection(img, gt_detections)
                    self.hal.frame_number += 1
                    f.close()
                    # To print FPS after each frame as been fully processed and displayed
                    end = time.time()
                    frame_time = round(end-start, 3)
                    fps = 1.0/frame_time
                    cv2.putText(img, "FPS: {}".format(int(fps)), (7,25), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,255,255), 1)
                    self.gui.showResult(img, str(len(gt_detections)))

                # Template specifics to run!
                finish_time = datetime.now()
                dt = finish_time - start_time
                ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

                self.iteration_counter += 1

                # The code should be run for at least the target time step
                # If it's less put to sleep
                if (ms < self.time_cycle):
                    time.sleep((self.time_cycle - ms) / 1000.0)

            
            self.hal.frame_number = 0
            self.perform_benchmark()
            print("Process thread closed!")

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
    def execute_thread(self, message):
        # Keep checking until the thread is alive
        # The thread will die when the coming iteration reads the flag
        if (self.thread != None):
            while self.thread.is_alive() or self.measure_thread.is_alive():
                pass

        # Turn the flag down, the iteration has successfully stopped!
        self.reload = False
        # New thread execution
        self.measure_thread = threading.Thread(target=self.measure_frequency)
        if(message == "#infer"):
            self.thread = threading.Thread(target=self.process_dl_model)
        if(message == "#eval"):
            self.thread = threading.Thread(target= self.eval_dl_model)
        if(message == "#video_infer"):
            self.thread = threading.Thread(target= self.video_infer)
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
        if (message[:5] == "#stop"):
            self.reload = True
            return
        if (message[:6] == "#infer"):
            try:
                self.reload = True
                self.execute_thread(message[:6])
            except:
                pass
        if (message[:5] == "#eval"):
            try:
                self.reload = True
                self.execute_thread(message[:5])
            except:
                pass
        if (message[:7] == "#visual"):
            try:
                self.visualizeModel()
            except:
                pass
        if(message[:11] == "#save_model"):
            try:
                self.saveModel(message[11:])
            except:
                pass
        if(message[:11] == "#save_video"):
            try:
                self.saveVideo(message[11:])
            except:
                pass
        if(message[:12] == "#video_infer"):
            try:
                self.reload = True
                self.execute_thread(message[:12])
            except:
                pass

    # Function that gets called when the server is connected
    def connected(self, client, server):
        start_console()
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
