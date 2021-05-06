import numpy as np
class Pose:
    def __init__(self):
        self.actual_pose = None
        self.pred_pose = None
        self.actual_path = []
        self.actual_path_time = []
        self.pred_path = []
        self.pred_path_time = [] 
        self.actual_path_without_interp =[]

    def set_pred_pose(self, pos,t):
        self.pred_pose = pos
        self.pred_path.append(pos)
        self.pred_path_time.append(t)

    def set_pred_path(self,path):
        self.pred_path = path

    def get_pred_pose(self):
        return (self.pred_pose)

    def set_actual_pose(self,pos,time):
        if len(self.actual_path) == 0:
            self.init_actual_pose = pos
            self.actual_pose = [0,0]
            self.actual_path.append([0,0])
        self.actual_pose = [pos[0] - self.init_actual_pose[0] , pos[1] - self.init_actual_pose[1]]
        self.actual_path_time.append(time)
        self.actual_path_without_interp.append(self.actual_pose)
        if len(self.pred_path_time) != 0:
            self.actual_path_x = np.interp(self.pred_path_time, self.actual_path_time, [k[0] for k in self.actual_path_without_interp])
            self.actual_path_y = np.interp(self.pred_path_time, self.actual_path_time, [k[1] for k in self.actual_path_without_interp])

            self.actual_path  = np.zeros((len(self.actual_path_x),2))
            self.actual_path[:,0] = self.actual_path_x
            self.actual_path[:,1] = self.actual_path_y
            

    def get_actual_pose(self):
        return (self.actual_pose)

    def get_actual_path(self):
        np_actual_path = np.array(self.actual_path)
        return (np_actual_path)

    def get_pred_path(self):
        np_pred_path = np.array(self.pred_path)
        return (np_pred_path)