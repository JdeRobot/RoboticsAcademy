import numpy as np
import math
import random
from math import pi as pi
import cv2

# User map canvas, pixel rows follow world x and pixel columns follow world y
MAP_HEIGHT = 970
MAP_WIDTH = 1500
SCALE_X = 23.58  # pixels per metre along world x
SCALE_Y = 23.53  # pixels per metre along world y
ORIGIN_X = -20.36  # world x of pixel row 0
ORIGIN_Y = -31.95  # world y of pixel column 0


def world_to_pixel(x, y):
    return SCALE_X * (x - ORIGIN_X), SCALE_Y * (y - ORIGIN_Y)


class Map:
    def __init__(self, pose_getter, noisy_pose_getter):
        self.pose_getter = pose_getter
        self.noisy_pose_getter = noisy_pose_getter

    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix(
            [
                [1, 0, 0, tx],
                [0, math.cos(angle), -math.sin(angle), ty],
                [0, math.sin(angle), math.cos(angle), tz],
                [0, 0, 0, 1],
            ]
        )
        return RT

    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix(
            [
                [math.cos(angle), 0, math.sin(angle), tx],
                [0, 1, 0, ty],
                [-math.sin(angle), 0, math.cos(angle), tz],
                [0, 0, 0, 1],
            ]
        )
        return RT

    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix(
            [
                [math.cos(angle), -math.sin(angle), 0, tx],
                [math.sin(angle), math.cos(angle), 0, ty],
                [0, 0, 1, tz],
                [0, 0, 0, 1],
            ]
        )
        return RT

    def RTVacuum(self):
        RTz = self.RTz(pi / 2, 50, 70, 0)
        return RTz

    def getRobotCoordinates(self):
        pose = self.pose_getter()
        x, y = world_to_pixel(pose.x, pose.y)
        return x, y, pose.yaw

    def getRobotCoordinatesWithNoise(self):
        pose = self.noisy_pose_getter()
        x, y = world_to_pixel(pose.x, pose.y)
        return x, y, pose.yaw

    # Function to reset
    def reset(self):
        # Nothing to do, service takes care!
        pass
