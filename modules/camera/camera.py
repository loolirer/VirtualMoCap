# Importing modules...
import numpy as np

from modules.camera.linear_model import *

class Camera:
    def __init__(self, 
                 resolution=(256,256), # Standard Coppelia resolution
                 fov_degrees=60, # Standard Coppelia fov
                 object_matrix=np.hstack((np.eye(3), np.zeros((3,1))))):
        
        self.resolution = resolution
        self.aspect_ratio = resolution[0]/resolution[1]
        self.fov_degrees = fov_degrees
        self.R, self.t = object_matrix[:, :-1], object_matrix[:, [-1]]
        
        self.intrinsic_matrix = build_intrinsic_matrix(fov_degrees, resolution)
        self.extrinsic_matrix = build_extrinsic_matrix(object_matrix)
        self.projection_matrix = build_projection_matrix(self.intrinsic_matrix, self.extrinsic_matrix)