
from ompl_planner.ompl_planner import ompl_planner
from control.control import noHolomonicControl
from path_processing.smoothPath import smooth
from map.occGridMap import occGridMap,costmap_2d

import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from math import cos,sin,tan,pow,pi,sqrt

class navigation:
    def __init__(self, files, origin_x, origin_y, resolution):
        self.control = noHolomonicControl()
        self.smooth = smooth()
        self.occGridMap = occGridMap()
        
        if files:
            self.have_map = True
            self.init_img = cv2.imread(files)
            size = self.init_img.shape
            self.gridMap = costmap_2d(size[0],size[1],resolution, origin_x, origin_y)
            for i in range(size[0]):
                for j in range(size[1]):
                    data = (np.sum(self.init_img[i,j]))/3
                    self.gridMap.setCost(i,j,data)
        else:
            self.have_map = False
            self.gridMap = costmap_2d(100,100,resolution, origin_x, origin_y)
        self.getTarget = False
    

    def update_map_autopark(self,car_pose,target_pose):

        #car [6,3]
        #target [8,5]
        p1 = self.move_pose(car_pose,[12,-3])
        p2 = self.move_pose(car_pose,[-12,-3])
        p3 = self.move_pose(car_pose,[12,3])
        p4 = self.move_pose(car_pose,[-12,3])
        p5 = self.move_pose(target_pose,[4,2.5])
        p6 = self.move_pose(target_pose,[-4,2.5])

        self.gridMap.resizeMapCheckPoint(p1)
        self.gridMap.resizeMapCheckPoint(p2)
        self.gridMap.resizeMapCheckPoint(p3)
        self.gridMap.resizeMapCheckPoint(p4)
        self.gridMap.resizeMapCheckPoint(p5)
        self.gridMap.resizeMapCheckPoint(p6)

        m_p1 = self.gridMap.worldToMapEnforceBounds(p1[0],p1[1])
        m_p2 = self.gridMap.worldToMapEnforceBounds(p2[0],p2[1])
        m_p3 = self.gridMap.worldToMapEnforceBounds(p3[0],p3[1])
        m_p4 = self.gridMap.worldToMapEnforceBounds(p4[0],p4[1])
        m_p5 = self.gridMap.worldToMapEnforceBounds(p5[0],p5[1])
        m_p6 = self.gridMap.worldToMapEnforceBounds(p6[0],p6[1])

        print ("getSizeInCellsX",self.gridMap.getSizeInCellsX())
        print ("getSizeInCellsY",self.gridMap.getSizeInCellsY())
        print ("getOriginX",self.gridMap.getOriginX())
        print ("getOriginY",self.gridMap.getOriginY())
        print ("p1",p1,m_p1)
        print ("p2",p2,m_p2)
        print ("p3",p3,m_p3)
        print ("p4",p4,m_p4)
        print ("p5",p5,m_p5)
        print ("p6",p6,m_p6)
        h = self.gridMap.size_x_
        w = self.gridMap.size_y_
        for i in range(h):
            for j in range(w):
                self.gridMap.setCost(i,j,-10)

        if m_p1[1] == m_p2[1]:
            #scene like autopark.world
            print ("update map")
            num_x_1 = m_p1[0] - m_p2[0]
            num_y_1 = m_p1[1] - m_p3[1]
            for i in range(num_x_1):
                for j in range(num_y_1):
                    self.gridMap.setCost(i+m_p4[0],j+m_p4[1],15)
            num_x_2 = m_p5[0] - m_p6[0]
            num_y_2 = m_p3[1] - m_p5[1]
            for i in range(num_x_2):
                for j in range(num_y_2):
                    self.gridMap.setCost(i+m_p6[0],j+m_p6[1],15)
            self.gridMap.drawMap()

        

    def move_pose(self,pose,move):
        assert len(pose) == 3
        assert len(move) == 2
        dx = move[1]*sin(pose[2]) + move[0]*cos(pose[2])
        dy = -move[1]*cos(pose[2]) + move[0]*sin(pose[2])
        x = pose[0] + dx
        y = pose[1] + dy
        return [x,y]
    
    def path_planning(self,current_pose,goal_pose):
        startX = current_pose[0]
        startY = current_pose[1]
        startYaw = current_pose[2]
        goalX = goal_pose[0]
        goalY = goal_pose[1]
        goalYaw = goal_pose[2]
        print ("pose",startX, startY, startYaw, goalX, goalY, goalYaw)

        start_pose_map = self.gridMap.worldToMap(startX,startY)
        goal_pose_map = self.gridMap.worldToMap(goalX, goalY)

        ompl_control = True
        ompl_sol = ompl_planner(self.gridMap, startX, startY, startYaw, goalX, goalY, goalYaw, "rrt", ompl_control, False)
        path_list = ompl_sol.omplRunOnce(50)
        #print self.pathlist
        if path_list :
            if ompl_control:
                path_list = self.path_smooth(path_list,current_pose)

                self.control.setPath(path_list)
                return True
            else:
                self.control.setControlYaw(False)
                self.control.setPath(path_list)
                return True
        else:
            return False
    
    def path_smooth(self,path_list,current_pose):
        plt.figure(1)
        plt.plot(path_list[0], path_list[1])
        num = len(path_list[0])
        for i in range(num):
            x1 = path_list[0][i]
            y1 = path_list[1][i]
            yaw = path_list[2][i]
            x2 = x1 + math.cos(yaw)
            y2 = y1 + math.sin(yaw)
            plt.plot([x1,x2],[y1,y2])
        plt.savefig("figure1.jpg") 
        self.smooth.setPath(path_list)
        path_list = self.smooth.Floyd(path_list)
        path_list = self.smooth.checkYaw(path_list, current_pose)

        plt.figure(2)
        plt.plot(path_list[0], path_list[1])

        num = len(path_list[0])
        for i in range(num):
            x1 = path_list[0][i]
            y1 = path_list[1][i]
            yaw = path_list[2][i]
            x2 = x1 + math.cos(yaw)
            y2 = y1 + math.sin(yaw)
            plt.plot([x1,x2],[y1,y2])
        plt.savefig("figure2.jpg")

        return path_list

    def path_following(self, current_pose):
        if self.getTarget:
            print "get Target"
            print self.control.getPath()
            return [0,0]
            
        Pose = [0,0,0]
        Pose[0] = current_pose[0]
        Pose[1] = current_pose[1]
        Pose[2] = current_pose[2]

        data = self.control.control(Pose,2)
        self.getTarget = self.control.isGetTarget()
        return data

    
    def path_check(self,path_smooth, costmap):
        a = 1
    

    def update_map(self, laser, id, pose):
        self.occGridMap.registerScan(pose,id,laser)
        self.occGridMap.drawMap()
