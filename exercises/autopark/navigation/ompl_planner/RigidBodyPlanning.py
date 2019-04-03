#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

from math import sin, cos
from functools import partial

from ompl import util as ou
from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og

from os.path import abspath, dirname, join
import numpy as np
import math

import matplotlib.pyplot as plt
## @cond IGNORE
# a decomposition is only needed for SyclopRRT and SyclopEST
class RigidBodyPlanning:
    def __init__(self, costMap, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, plannerType):
        self.costMap = costMap
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.plannerType = plannerType
        self.simpleSetupControl()


    def isStateValid(self, state):
        # perform collision checking or check if other constraints are
        # satisfied
        wx = state.getX()
        wy = state.getY()
        if not self.costMap:
            tmp = True
            if wy < -4 and wy > 5:
                tmp = False
            elif wy < -0.5:
                if wx > 12 or wx < 5.5:
                    tmp = False 
        else:
            mPoint = self.costMap.worldToMapEnforceBounds(wx, wy)
            cost = self.costMap.getCost(mPoint[0],mPoint[1])
            if cost < -5:
                tmp = False
            else:
                tmp = True
        return tmp
    
    def simpleSetupControl(self):
        self.setSpace()
        self.setProblem()
        self.setPlanner()

    def setSpace(self):
        # construct the state space we are planning in
        self.space = ob.SE2StateSpace()

        # set the bounds for the R^2 part of SE(2)
        self.bounds = ob.RealVectorBounds(2)
        if not self.costMap:
            self.bounds.setLow(-8)
            self.bounds.setHigh(20)
        else:
            ox = self.costMap.getOriginX()
            oy = self.costMap.getOriginY()
            size_x = self.costMap.getSizeInMetersX()
            size_y = self.costMap.getSizeInMetersY()
            low = min(ox, oy)
            high = max(ox+size_x, oy+size_y)
            print ("low",low)
            print ("high",high)
            self.bounds.setLow(low)
            self.bounds.setHigh(high)
        self.space.setBounds(self.bounds)

        # define a simple setup class
        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))

    def setProblem(self):
        # create a start state
        start = ob.State(self.space)
        start().setX(self.start_x)
        start().setY(self.start_y)
        start().setYaw(self.start_yaw)

        # create a goal state
        goal = ob.State(self.space)
        goal().setX(self.goal_x)
        goal().setY(self.goal_y)
        goal().setYaw(self.goal_yaw)

        # set the start and goal states
        self.ss.setStartAndGoalStates(start, goal, 0.05)
    
    def setPlanner(self):
        self.si = self.ss.getSpaceInformation()
        if self.plannerType.lower() == "bitstar":
            planner = og.BITstar(self.si)
        elif self.plannerType.lower() == "fmtstar":
            planner = og.FMT(self.si)
        elif self.plannerType.lower() == "informedrrtstar":
            planner = og.InformedRRTstar(self.si)
        elif self.plannerType.lower() == "prmstar":
            planner = og.PRMstar(self.si)
        elif self.plannerType.lower() == "rrtstar":
            planner = og.RRTstar(self.si)
        elif self.plannerType.lower() == "sorrtstar":
            planner = og.SORRTstar(self.si)
        else:
            OMPL_ERROR("Planner-type is not implemented in allocation function.")
            planner = og.RRTstar(self.si)
        self.ss.setPlanner(planner)

    def solve(self, runtime = None ):
        if not runtime:
            runtime = 1
        # attempt to solve the problem
        solved = self.ss.solve(runtime)

        if solved:
            self.ss.simplifySolution()
            # print the path to screen
            p = self.ss.getSolutionPath()
            print("Found solution:\n%s" % self.ss.getSolutionPath().printAsMatrix())
            #p.interpolate()
            pathlist = [[] for i in range(3)]

            for i in range(p.getStateCount()):
                x = p.getState(i).getX()
                y = p.getState(i).getY()
                yaw = p.getState(i).getYaw()
                pathlist[0].append(x)
                pathlist[1].append(y)
                pathlist[2].append(yaw)
            return pathlist
        else:
            print ("can't find path")
            return 0


if __name__ == "__main__":
    
    
    planner = RigidBodyPlanning(None, 6,3,0,7.25,-3,0, 'rrtstar')
    pathlist = planner.solve()

    plt.figure(1)
    plt.plot(pathlist[0], pathlist[1])
    num = len(pathlist[0])
    for i in range(num):
        x1 = pathlist[0][i]
        y1 = pathlist[1][i]
        yaw = pathlist[2][i]
        x2 = x1 + 0.5*cos(yaw)
        y2 = y1 + 0.5*sin(yaw)
        plt.plot([x1,x2],[y1,y2])
    plt.show()