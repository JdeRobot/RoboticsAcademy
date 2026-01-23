#!/usr/bin/env python


class Beacon:
    def __init__(self, id, pose, active=False, reached=False):
        self.id = id
        self.pose = pose
        self.active = active
        self.reached = reached

    def get_pose(self):
        return self.pose

    def get_id(self):
        return self.id

    def is_reached(self):
        return self.reached

    def set_reached(self, value):
        self.reached = value

    def is_active(self):
        return self.active

    def set_active(self, value):
        self.active = value
