#!/usr/bin/env python
from ompl import util as ou
from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og
from os.path import abspath, dirname, join
import sys
from functools import partial

import numpy as np
from math import sin, cos, tan
import matplotlib.pyplot as plt

from RigidBodyPlanning import RigidBodyPlanning
from RigidBodyPlanningWithControls import RigidBodyPlanningWithControls
from RigidBodyPlanningWithODESolverAndControls import RigidBodyPlanningWithODESolverAndControls


class ompl_planner:
    def __init__(self, costMap, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, plannerType, control = True, ODESolver = False):
        self.costMap = costMap
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.plannerType = plannerType
        self.control = control
        self.ODESolver = ODESolver
    
    def setMap(self, costMap):
        self.costMap = costMap

    def setPose(self,start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
    
    def setPlannerType(self, plannerType):
        self.plannerType = plannerType
    
    def setControl(self, control):
        self.control = control

    def setODESolver(self, ODESolver):
        self.ODESolver = ODESolver
    
    def omplRunOnce(self, runtime=None):
        if self.control :
            if self.ODESolver:
                planner = RigidBodyPlanningWithODESolverAndControls(self.costMap, self.start_x, self.start_y, self.start_yaw, self.goal_x, self.goal_y, self.goal_yaw, self.plannerType)
            else:
                planner = RigidBodyPlanningWithControls(self.costMap, self.start_x, self.start_y, self.start_yaw, self.goal_x, self.goal_y, self.goal_yaw, self.plannerType)
        else:
            planner = RigidBodyPlanning(self.costMap, self.start_x, self.start_y, self.start_yaw, self.goal_x, self.goal_y, self.goal_yaw, self.plannerType)
        
        if not runtime:
            pathlist = planner.solve()
        else:
            pathlist = planner.solve(runtime)
        
        return pathlist
        