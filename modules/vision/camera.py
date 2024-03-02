# Importing modules...
import numpy as np
import cv2

from modules.vision.linear_model import *

class Camera:
    def __init__(self, 
                 vision_sensor_handle=-1, # Coppelia's World handle 
                 resolution=(256,256), # Standard Coppelia resolution
                 fov_degrees=60, # Standard Coppelia fov
                 object_matrix=np.hstack((np.eye(3), np.zeros((3,1))))):
        
        self.vision_sensor_handle = vision_sensor_handle

        self.resolution = resolution
        self.aspect_ratio = resolution[0]/resolution[1]
        self.fov_degrees = fov_degrees
        self.R, self.t = object_matrix[:, :-1], object_matrix[:, [-1]]
        
        self.intrinsic_matrix = build_intrinsic_matrix(fov_degrees, resolution)
        self.extrinsic_matrix = build_extrinsic_matrix(object_matrix)
        self.projection_matrix = build_projection_matrix(self.intrinsic_matrix, self.extrinsic_matrix)

    def get_coppelia_image(self, api_method):
        image_raw, resolution_x, resolution_y = api_method(self.vision_sensor_handle)
        image_unflipped = np.frombuffer(image_raw, dtype=np.uint8).reshape(resolution_y, resolution_x, 3) # 3 Channel image buffer 

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # This is consistent with the axes of vision sensors, pointing Z outwards, Y up
        image = cv2.flip(image_unflipped, 0)

        return image