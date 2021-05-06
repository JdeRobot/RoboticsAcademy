import rosbag
from genpy.message import strify_message
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import OrderedDict
class Data:
    def __init__(self ,imu , orientation , color_img , depth_img , imu_t , orientation_t , color_img_t , depth_img_t, scan ,scan_t,topic = None):
        self.sensor = topic
        self.accelerometer = imu
        self.orientation = orientation
        self.color_img = color_img
        self.depth_img = depth_img
        self.accelerometer_t = imu_t
        self.orientation_t = orientation_t 
        self.color_img_t = color_img_t
        self.depth_img_t = depth_img_t
        self.scan = scan
        self.scan_t = scan_t
class Read_Rosbag:
    def __init__(self,pose_obj , file_dir,myGUI):
        self.bag = rosbag.Bag(file_dir)
        self.init_read = self.bag.read_messages()
        self.ground_truth = OrderedDict()
        self.ground_truth_time = []
        self.myGUI = myGUI
        self.pose_obj = pose_obj
        self.init_timestamp = self.bag.get_start_time()
        self.generator = self.bag.read_messages()
        
        self.bridge = CvBridge()
        self.pose = {}
        self.imu = {}
        self.color_img = None
        self.current_timestamp = None
        self.depth_img = None
        orientation_count = self.bag.get_message_count('/pose') + 1
        self.orientation_noise = np.random.normal(0, (1/3), [orientation_count,2]).tolist() #orientation noise mu =0 and 5*sigma = 1
    def align(self,model,data):
        """Align two trajectories using the method of Horn (closed-form) 
           and compute translational error.
        
        Input:
        model -- first trajectory (2xn1)
        data -- second trajectory (2xn2)
        
        Output:
        rot -- rotation matrix (3x3)
        trans -- translation vector (3x1)
        trans_error -- translational error per point (1xn)
        
        """
        model = model.T
        data = data.T

        if model.shape[1] > data.shape[1] :
            diff = model.shape[1] - data.shape[1]
            #remove = np.random.choice(range(model.shape[1]) , size = diff , replace =False)
            remove = np.linspace(start = 1 , stop = model.shape[1] , num = diff , endpoint=False ,dtype= int)         
            model = np.delete(model , remove , axis = 1)
            z=np.zeros((1,data.shape[1]))
            data= np.vstack((data,z)) 
            model= np.vstack((model,z))
        elif model.shape[1] < data.shape[1] :
            diff = -(model.shape[1] - data.shape[1])
            #remove = np.random.choice(range(data.shape[1]) , size = diff  ,replace = False)
            remove = np.linspace(start = 1 , stop = data.shape[1] , num = diff , endpoint=False ,dtype= int)         
            data = np.delete(data , remove , axis = 1)
            z=np.zeros((1,model.shape[1]))
            data= np.vstack((data,z))
            model= np.vstack((model,z))
        else:
            z=np.zeros((1,data.shape[1]))
            data= np.vstack((data,z)) 
            model= np.vstack((model,z))


        model_zerocentered = model - model.mean(1).reshape((model.mean(1).shape[0]), 1)
        data_zerocentered = data - data.mean(1).reshape((data.mean(1).shape[0]), 1)
        
        W = np.zeros( (3,3) )
        for column in range(model.shape[1]):
            W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
        U,d,Vh = np.linalg.linalg.svd(W.transpose())
        S = np.matrix(np.identity( 3 ))
        if(np.linalg.det(U) * np.linalg.det(Vh)<0):
            S[2,2] = -1
        rot = U*S*Vh
        trans = (data.mean(1).reshape((data.mean(1).shape[0]), 1)) - rot * (model.mean(1).reshape((model.mean(1).shape[0]), 1))
        
        model_aligned = rot * model + trans
        alignment_error = model_aligned - data
        
        trans_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]
            
        return trans_error
    def depthToRGB8(self,float_img_buff, encoding):
        '''
        Translates from Distance Image format to RGB. Inf values are represented by NaN, when converting to RGB, NaN passed to 0 

        @param float_img_buff: ROS Image to translate

        @type img: ros image

        @return a Opencv RGB image

        '''
        gray_image = None
        if (encoding[-3:-2]== "U"):
            gray_image = float_img_buff
        else:    
            float_img = np.zeros((float_img_buff.shape[0], float_img_buff.shape[1], 1), dtype = "float32")
            float_img.data = float_img_buff.data
            gray_image=cv2.convertScaleAbs(float_img, alpha=255/8)

        #cv_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)

        return gray_image

    def next_message(self , sensors):

        self.pose = {}
        self.imu = {}
        self.imu_t = None
        self.orientation = {}
        self.orientation_t =None
        self.color_img = None
        self.color_img_t = None
        self.depth_img = None
        self.depth_img_t = None
        self.pose_t = None
        self.scan = None
        self.scan_t = None
        while True:
            try:
                topic, msg, t = next(self.generator)
            except StopIteration:
                pred_path = self.pose_obj.get_pred_path()
                actual_path = self.pose_obj.get_actual_path()
                trans_error = self.align(pred_path , actual_path)
                self.myGUI.median.display(np.median(trans_error))
                self.myGUI.mean.display(np.mean(trans_error))
                self.myGUI.Std_dev.display(np.std(trans_error))
                self.myGUI.rmse.display(np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
                print("End")
            t = t.to_sec()
            if topic == '/imu' and ('accelerometer' in sensors or sensors == () or 'stream' in sensors):
                self.imu['x'] = float(strify_message(msg.linear_acceleration.x))
                self.imu['y'] = float(strify_message(msg.linear_acceleration.z))
                self.imu_t = t
                self.topic_name = 'accelerometer'

            elif topic == '/pose':
                self.pose['x'] = float(strify_message(msg.pose.pose.position.x))
                self.pose['y'] = float(strify_message(msg.pose.pose.position.y))
                self.pose_t = t 
                if ('orientation' in sensors or sensors == () or 'stream' in sensors):
                    self.orientation['qz'] = float(strify_message(msg.pose.pose.orientation.z))
                    self.orientation['qw'] = float(strify_message(msg.pose.pose.orientation.w))
                    noise = self.orientation_noise.pop()
                    self.orientation['qz'] = self.orientation['qz'] + noise[0]
                    self.orientation['qw'] = self.orientation['qw'] + noise[1]
                    self.orientation_t = t
                    self.topic_name  = 'orientation'

            elif topic == '/camera/rgb/image_color' and ('color_img' in sensors or sensors == () or 'stream' in sensors):
                self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self.color_img_t = t
                self.topic_name = 'color_img'

            elif topic == '/camera/depth/image' and ('depth_img' in sensors or sensors == () or 'stream' in sensors):
                gray_img_buff = self.bridge.imgmsg_to_cv2(msg,  desired_encoding=msg.encoding)
                
                #self.depth_img = self.depthToRGB8(gray_img_buff , msg.encoding)
                self.depth_img = gray_img_buff
                self.depth_img_t = t
                self.topic_name = 'depth_img'

            elif topic == '/scan' and ('scan' in sensors or sensors == () or 'stream' in sensors):
                self.scan = np.array(msg.ranges)
                self.scan_t = t
                self.topic_name = 'scan'

            self.current_timestamp = t
            
            if 'stream' in sensors:
                break

            if sensors == () :
                if len(self.pose) > 0 and len(self.imu) > 0 and self.color_img != None and self.depth_img != None and self.scan == None:
                    break
            else:
                if 'accelerometer' not in sensors:
                    k= True
                else:
                    if self.imu_t == None:
                        k = False
                    else:
                        k = True

                if 'orientation' not in sensors:
                    l= True
                else:
                    if self.orientation_t == None:
                        l= False
                    else:
                        l = True

                if 'color_img' not in sensors:
                    m= True
                else:
                    if self.color_img_t == None:
                        m= False
                    else:
                        m = True

                if 'depth_img' not in sensors:
                    n= True
                else:
                    if self.depth_img_t == None:
                        n= False
                    else:
                        n = True

                if 'scan' not in sensors:
                    o=True
                else:
                    if self.scan_t == None:
                        o = False
                    else:
                        o = True

                if k and l and m and n and o:
                    break

    def time_elapsed(self):

        return (self.current_timestamp - self.init_timestamp)

    def getData(self ,sensors):
        self.next_message(sensors)
        if self.pose != {}:
            self.pose_obj.set_actual_pose([self.pose['x'] , self.pose['y']] , self.pose_t)
        if 'stream' in sensors:
            data = Data(self.imu ,self.orientation , self.color_img , self.depth_img ,self.imu_t ,self.orientation_t , self.color_img_t , self.depth_img_t , self.scan , self.scan_t, self.topic_name)
        else:
            data = Data(self.imu ,self.orientation , self.color_img , self.depth_img ,self.imu_t ,self.orientation_t , self.color_img_t , self.depth_img_t, self.scan , self.scan_t)
        return data