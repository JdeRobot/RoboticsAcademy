from os import path
from BoundingBox import BoundingBox
from BoundingBoxes import BoundingBoxes
from utils import *


def getBoundingBoxes():
    """Read txt files containing bounding boxes (ground truth and detections)."""
    allBoundingBoxes = BoundingBoxes()
    import glob
    import os
    # Read ground truths
    currentPath = os.path.dirname(os.path.abspath(__file__))
    folderGT = os.path.join(currentPath, 'groundtruths')
    os.chdir(folderGT)
    files = glob.glob("*.txt")
    files.sort()
    # Class representing bounding boxes (ground truths and detections)
    allBoundingBoxes = BoundingBoxes()
    # Read GT detections from txt file
    # Each line of the files in the groundtruths folder represents a ground truth bounding box
    # (bounding boxes that a detector should detect) 
    # Each value of each line is  "class_id, x_min, y_min, x_max, y_max" respectively
    # Class_id represents the class of the bounding box, which in this exercise is "person"
    # x_min, y_min represents the most top-left coordinates of the bounding box
    # x_max, y_max represents the most bottom-right coordinates of the bounding box
    for f in files:
        nameOfImage = f.replace(".txt", "")
        fh1 = open(f, "r")
        for line in fh1:
            line = line.replace("\n", "")
            if line.replace(' ', '') == '':
                continue
            splitLine = line.split(" ")
            idClass = splitLine[0]  # class
            x_min = float(splitLine[1]) 
            y_min = float(splitLine[2])
            x_max = float(splitLine[3])
            y_max = float(splitLine[4])
            bb = BoundingBox(
                nameOfImage,
                idClass,
                x_min,
                y_min,
                x_max,
                y_max,
                CoordinatesType.Absolute, (0,0),
                BBType.GroundTruth,
                format=BBFormat.XYWH)
            allBoundingBoxes.addBoundingBox(bb)
        fh1.close()
    # Read detections
    folderDet = os.path.join(currentPath, 'detections')
    os.chdir(folderDet)
    files = glob.glob("*.txt")
    files.sort()
    # Read detections from txt file
    # Each line of the files in the detections folder represents a detected bounding box.
    # Each value of each line is  "class_id, confidence, x_min, y_min, x_max, y_max" respectively
    # Class_id represents the class of the detected bounding box, which in this exercise is person
    # Confidence represents confidence (from 0 to 1) that this detection belongs to the class_id.
    # x_min, y_min represents the most top-left coordinates of the bounding box
    # x_max, y_max represents the most bottom-right coordinates of the bounding box
    for f in files:
        nameOfImage = f.replace(".txt", "")
        # Read detections from txt file
        fh1 = open(f, "r")
        for line in fh1:
            line = line.replace("\n", "")
            if line.replace(' ', '') == '':
                continue
            splitLine = line.split(" ")
            idClass = splitLine[0]  # class
            confidence = float(splitLine[1])  # confidence
            x_min = float(splitLine[2])
            y_min = float(splitLine[3])
            x_max = float(splitLine[4])
            y_max = float(splitLine[5])
            bb = BoundingBox(
                nameOfImage,
                idClass,
                x_min,
                y_min,
                x_max,
                y_max,
                CoordinatesType.Absolute, (0,0),
                BBType.Detected,
                confidence,
                format=BBFormat.XYWH)
            allBoundingBoxes.addBoundingBox(bb)
        fh1.close()

    return allBoundingBoxes


