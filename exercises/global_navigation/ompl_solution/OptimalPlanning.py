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

# Author: Luis G. Torres, Mark Moll

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))),'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from math import sqrt
from sys import argv
import argparse
import cv2
## @cond IGNORE
# Our "collision checker". For this demo, our robot's state space
# lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
# centered at (0.5,0.5). Any states lying in this circular region are
# considered "in collision".
class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, mapIm):
        super(ValidityChecker, self).__init__(si)
        self.mapIm = mapIm

    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        x = int(state[0])
        y = int(state[1])
        if self.mapIm[x,y]==0 :
            return False
        else:
            return True
        #return self.clearance(state) > 0.0

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        # Extract the robot's (x,y) position from its state
        x = int(state[0])
        y = int(state[1])
        if self.mapIm[x,y]==0 :
            return -1
        else:
            return 1

        # Distance formula between two points, offset by the circle's
        # radius
        #return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25


## Defines an optimization objective which attempts to steer the
#  robot away from obstacles. To formulate this objective as a
#  minimization of path cost, we can define the cost of a path as a
#  summation of the costs of each of the states along the path, where
#  each state cost is a function of that state's clearance from
#  obstacles.
#
#  The class StateCostIntegralObjective represents objectives as
#  summations of state costs, just like we require. All we need to do
#  then is inherit from that base class and define our specific state
#  cost function by overriding the stateCost() method.
#
class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / self.si_.getStateValidityChecker().clearance(s))


class optimalPlanning():
    def __init__(self):
        self.startPose = [0,0]
        self.goalPose = [0,0]


    ## Returns a structure representing the optimization objective to use
    #  for optimal motion planning. This method returns an objective
    #  which attempts to minimize the length in configuration space of
    #  computed paths.
    def getPathLengthObjective(self, si):
        return ob.PathLengthOptimizationObjective(si)

    ## Returns an optimization objective which attempts to minimize path
    #  length that is satisfied when a path of length shorter than 1.51
    #  is found.
    def getThresholdPathLengthObj(self, si):
        obj = ob.PathLengthOptimizationObjective(si)
        obj.setCostThreshold(ob.Cost(1.51))
        return obj

    ## Return an optimization objective which attempts to steer the robot
    #  away from obstacles.
    def getClearanceObjective(self,si):
        return ClearanceObjective(si)

    ## Create an optimization objective which attempts to optimize both
    #  path length and clearance. We do this by defining our individual
    #  objectives, then adding them to a MultiOptimizationObjective
    #  object. This results in an optimization objective where path cost
    #  is equivalent to adding up each of the individual objectives' path
    #  costs.
    #
    #  When adding objectives, we can also optionally specify each
    #  objective's weighting factor to signify how important it is in
    #  optimal planning. If no weight is specified, the weight defaults to
    #  1.0.
    def getBalancedObjective1(self,si):
        lengthObj = ob.PathLengthOptimizationObjective(si)
        clearObj = ClearanceObjective(si)

        opt = ob.MultiOptimizationObjective(si)
        opt.addObjective(lengthObj, 5.0)
        opt.addObjective(clearObj, 1.0)

        return opt

    ## Create an optimization objective equivalent to the one returned by
    #  getBalancedObjective1(), but use an alternate syntax.
    #  THIS DOESN'T WORK YET. THE OPERATORS SOMEHOW AREN'T EXPORTED BY Py++.
    # def getBalancedObjective2(si):
    #     lengthObj = ob.PathLengthOptimizationObjective(si)
    #     clearObj = ClearanceObjective(si)
    #
    #     return 5.0*lengthObj + clearObj


    ## Create an optimization objective for minimizing path length, and
    #  specify a cost-to-go heuristic suitable for this optimal planning
    #  problem.
    def getPathLengthObjWithCostToGo(self,si):
        obj = ob.PathLengthOptimizationObjective(si)
        obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
        return obj


    # Keep these in alphabetical order and all lower case
    def allocatePlanner(self,si, plannerType):
        if plannerType.lower() == "bfmtstar":
            return og.BFMT(si)
        elif plannerType.lower() == "bitstar":
            return og.BITstar(si)
        elif plannerType.lower() == "fmtstar":
            return og.FMT(si)
        elif plannerType.lower() == "informedrrtstar":
            return og.InformedRRTstar(si)
        elif plannerType.lower() == "prmstar":
            return og.PRMstar(si)
        elif plannerType.lower() == "rrtstar":
            return og.RRTstar(si)
        elif plannerType.lower() == "sorrtstar":
            return og.SORRTstar(si)
        else:
            OMPL_ERROR("Planner-type is not implemented in allocation function.");


    # Keep these in alphabetical order and all lower case
    def allocateObjective(self,si, objectiveType):
        if objectiveType.lower() == "pathclearance":
            return self.getClearanceObjective(si)
        elif objectiveType.lower() == "pathlength":
            return self.getPathLengthObjective(si)
        elif objectiveType.lower() == "thresholdpathlength":
            return self.getThresholdPathLengthObj(si)
        elif objectiveType.lower() == "weightedlengthandclearancecombo":
            return self.getBalancedObjective1(si)
        else:
            OMPL_ERROR("Optimization-objective is not implemented in allocation function.");

    def setStart(self,poseX,poseY):
        # Set our robot's starting state to be the bottom-left corner of
        # the environment, or (0,0).
        self.startPose[0] = poseX
        self.startPose[1] = poseY
    
    def setGoal(self,poseX,poseY):
        # Set our robot's goal state to be the top-right corner of the
        # environment, or (1,1).
        self.goalPose[0] = poseX
        self.goalPose[1] = poseY

    def SetMap(self,mapIm):
        self.mapIm = mapIm
        size = mapIm.shape
        h = size[0]  
        w = size[1]
        for i in range(h):
            for j in range(w):
                if self.mapIm[i,j] > 250:
                    self.mapIm[i,j] = 255
                else:
                    self.mapIm[i,j] = 0
        print self.mapIm[200,200]
        cv2.imshow('image',self.mapIm)
        


    def plan(self, runTime, plannerType, objectiveType, fname):
        # Construct the robot state space in which we're planning. We're
        # planning in [0,1]x[0,1], a subset of R^2.
        self.space = ob.RealVectorStateSpace(2)

        # Set the bounds of space to be in [0,1].
        self.space.setBounds(0, 400.0)

        # Construct a space information instance for this state space
        self.si = ob.SpaceInformation(self.space)

        # Set the object used to check which states in the space are valid
        validityChecker = ValidityChecker(self.si, self.mapIm)
        self.si.setStateValidityChecker(validityChecker)

        self.si.setup()

        
        self.start = ob.State(self.space)
        self.start[0] = self.startPose[0]
        self.start[1] = self.startPose[1]

        
        self.goal = ob.State(self.space)
        self.goal[0] = self.goalPose[0]
        self.goal[1] = self.goalPose[1]

        # Create a problem instance
        self.pdef = ob.ProblemDefinition(self.si)

        # Set the start and goal states
        self.pdef.setStartAndGoalStates(self.start, self.goal)

        # Create the optimization objective specified by our command-line argument.
        # This helper function is simply a switch statement.
        self.pdef.setOptimizationObjective(self.allocateObjective(self.si, objectiveType))

        # Construct the optimal planner specified by our command line argument.
        # This helper function is simply a switch statement.
        self.optimizingPlanner = self.allocatePlanner(self.si, plannerType)

        # Set the problem instance for our planner to solve
        self.optimizingPlanner.setProblemDefinition(self.pdef)
        self.optimizingPlanner.setup()

        # attempt to solve the planning problem in the given runtime
        solved = self.optimizingPlanner.solve(runTime)

        if solved:
            # Output the length of the path found
            print("{0} found solution of path length {1:.4f} with an optimization objective value of {2:.4f}".format(self.optimizingPlanner.getName(), self.pdef.getSolutionPath().length(), self.pdef.getSolutionPath().cost(self.pdef.getOptimizationObjective()).value()))
            print self.pdef.getSolutionPath()
            # If a filename was specified, output the path as a matrix to
            # that file for visualization
            # if fname:
            #     with open(fname,'w') as outFile:
            #         outFile.write(pdef.getSolutionPath().printAsMatrix())
        else:
            print("No solution found.")

#     # Add a filename argument
#     parser.add_argument('-t', '--runtime', type=float, default=1.0, help='(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
#     parser.add_argument('-p', '--planner', default='RRTstar', choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', 'SORRTstar'], help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.') # Alphabetical order
#     parser.add_argument('-o', '--objective', default='PathLength', choices=['PathClearance', 'PathLength', 'ThresholdPathLength', 'WeightedLengthAndClearanceCombo'], help='(Optional) Specify the optimization objective, defaults to PathLength if not given.') # Alphabetical order
#     parser.add_argument('-f', '--file',  default=None, help='(Optional) Specify an output path for the found solution path.')
#     parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to WARN.')
