from gui import GUI
from hal import HAL
from cv2 import cv2
import numpy as np
import time
import math


# PID class
class PID:
    def __init__(self, kp, kd, ki, st):

        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_error = 0
        self.last_output = 0
        self.last_time = 0
        self.st = st
        self.last_sample = time.time()
        self.iterm = 0

    def set_lims(self, outmin, outmax):
        self.outmax = outmax
        self.outmin = outmin

    def calc(self, error):

        out = self.last_output

        # Calculate the time which has passed
        diff_time = time.time() - self.last_sample

        if (diff_time >= self.st):
            # Derivative part
            diff_error = error - self.last_error

            # Integrative part (never higher than max)
            self.iterm += error * self.ki
            if (self.iterm > self.outmax):
                self.iterm = self.outmax
            elif (self.iterm < self.outmin):
                self.iterm = self.outmin

            # Output (never higher than max)
            out = self.kp * error + self.kd * diff_error + self.iterm
            if (out > self.outmax):
                out = self.outmax
            elif (out < self.outmin):
                out = self.outmin

            # Store info needed for next time
            self.last_error = error
            self.last_output = out
            self.last_sample = time.time()

        return out


class ErrBuff:
    def __init__(self, size):
        self.size = size
        self.ac_error = []
        self.nelems = 0
        self.next = 0

    def add(self, error):

        # Check if buffer has to cycle
        if self.next == self.size: self.next = 0

        # Checks if already big enough
        if self.nelems < self.size:
            self.ac_error.append(error)
            self.nelems += 1
        else:
            self.ac_error[self.next] = error

        self.next += 1

    def get_mean(self):
        mean = 0
        # print(self.ac_error)
        if (self.nelems > 0): mean = sum(self.ac_error) / self.nelems

        return mean


# Color filter
red_mask = ([17, 15, 70], [50, 56, 255])
center_offset = 20
center_margin = 10
black_pixel = np.array([0, 0, 0])

# PID variables
direct = 0

# Angular pid
sp1 = 320
kp_1 = 0.01
kd_1 = 0.026
ki_1 = 0.00011
outmax_1 = 3
outmin_1 = -3.5

# Linear pid
sp2 = center_offset + int(center_margin / 2)
kp_2 = 0.04
kd_2 = 0.08
ki_2 = 0
outmax_2 = 6
outmin_2 = -6

# Car variables
max_linear = 11
max_lin_dec = 6
error_thres = 25

# PIDS objects (angular and linear speed)
pid1 = PID(kp_1, kd_1, ki_1, 0.03)
pid1.set_lims(outmin_1, outmax_1)

# buffer object
buff = ErrBuff(15)


# Apply the color mask to the raw img and return the filtered image
def filter_img(raw_image, c_mask):
    lower = np.array(c_mask[0], dtype="uint8")
    upper = np.array(c_mask[1], dtype="uint8")

    mask = cv2.inRange(raw_image, lower, upper)
    f_img = cv2.bitwise_and(raw_image, raw_image, mask=mask)

    return f_img


# Gets a reference position between (center+offset, center+offset+margin)
def get_line_ref(img, offset, margin):
    height = img.shape[0]
    width = img.shape[1]

    center_row = int(height / 2) + offset

    c_x = 0
    c_y = 0
    npixels = 0

    for x in range(width):
        for y in range(center_row, center_row + margin):
            # Get pixel val and compare it with values for black
            pixel_val = img[y][x]
            comparison = (pixel_val == black_pixel)

            if not comparison.all():
                c_x += x
                c_y += y
                npixels += 1

    if (npixels > 0):
        c_x /= npixels
        c_y /= npixels

    return (int(c_x), int(c_y))


# Shows a debug image with the reference point and the set point
def show_debug_img(img, ref):
    dimensions = img.shape
    height = dimensions[0]
    width = dimensions[1]

    set_point = (int(width / 2), int(height / 2))

    img = cv2.circle(img, ref, 4, (255, 255, 0), -1)
    # img = cv2.circle(img, set_point, 5, (0, 255, 255), -1)
    img = cv2.line(img, (set_point[0], 0), (set_point[0], height), (0, 255, 0), thickness=1)

    GUI.showImage(img)




raw_img = HAL.getImage()
f_img = filter_img(raw_img, red_mask)

# The reference used for angular speed calculation
ref1 = get_line_ref(f_img, center_offset, center_margin)

if (ref1 != (0, 0)):

    # Error calculation
    ref1_x = ref1[0]
    error = sp1 - ref1_x
    buff.add(abs(error))
    mean_error = round(buff.get_mean(), 2)

    norm_mean = mean_error / error_thres
    if (norm_mean > 1): norm_mean = 1

    linear_speed = max_linear - max_lin_dec * norm_mean
    angular_speed = pid1.calc(error)

    # Control action
    HAL.setW(angular_speed)
    HAL.setV(linear_speed)
else:
    HAL.setW(0)
    HAL.setV(0)

show_debug_img(f_img, ref1)