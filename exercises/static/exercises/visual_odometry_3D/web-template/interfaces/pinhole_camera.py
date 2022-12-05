import numpy as np
import yaml


class PinholeCamera:  
    
    def __init__(self, width=None, height=None, fu=None, fv=None, cu=None, cv=None,
        distortion_model=None, distortion_coefficients=None, extrinsics=None, intrinsics=None):

        self.width = width
        self.height = height
        self.fu = fu
        self.fv = fv
        self.cu = cu
        self.cv = cv
        self.distortion_model = distortion_model
        self.distortion_coefficients = distortion_coefficients
        self.extrinsics = extrinsics
        self.intrinsics = intrinsics
        self.camera_matrix = self.cameraMatrix()


    def get_camera_matrix(self):
        return np.array([[self.fu, 0, self.cu], [0, self.fv, self.cv], [0, 0, 1]])


    def print(self):
        
        print("PinholeCamera:")
        print("  width:", self.width)
        print("  height:", self.height)
        print("  fu:", self.fu)
        print("  fv:", self.fv)
        print("  cu:", self.cu)
        print("  cv:", self.cv)
        print("  distortion_model:", self.distortion_model)
        print("  distortion_coefficients:", self.distortion_coefficients)
        print("  extrinsics:", self.extrinsics)
        print("  intrinsics:", self.intrinsics)


    @classmethod
    def from_euromav(cls, file_path: str):

        with open(file_path, "r") as f:
            try:
                data = yaml.load(f, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        width, height = data["resolution"]
        fu, fv, cu, cv = data["intrinsics"]
        distortion_model = data["distortion_model"]
        distortion_coefficients = np.array(data["distortion_coefficients"])
        extrinsics = np.array(data["T_BS"]["data"]).reshape(4, 4)
        intrinsics = np.array(data["intrinsics"])

        return cls(width, height, fu, fv, cu, cv, distortion_model, distortion_coefficients, extrinsics, intrinsics)


    @classmethod
    def from_kitti(cls, file_path: str, width: int, height: int):

        with open(file_path, "r") as f:
            data = f.readlines()
            for i in range(len(data)):
                data[i] = data[i].replace("\n", "").split(" ")

        fu = float(data[0][1])
        fv = float(data[0][1])
        cu = float(data[0][3])
        cv = float(data[0][7])

        return cls(width, height, fu, fv, cu, cv)


    @classmethod
    def from_vkitti2(cls, file_path: str, width: int, height: int, camera: int):
        
        with open(file_path, "r") as f:
            lines = f.readlines()
        
        if camera == 0:
            data = lines[1].replace("\n", "").split(" ")
        else:
            data = lines[2].replace("\n", "").split(" ")
        
        fu = float(data[2])
        fv = float(data[3])
        cu = float(data[4])
        cv = float(data[5])

        return cls(width, height, fu, fv, cu, cv)
