# Importing modules...
import numpy as np
import cv2

from modules.vision.blob_detection import *
from modules.vision.linear_projection import *
from modules.vision.lens_distortion import *
from modules.vision.image_noise import *

class Camera:
    def __init__(self, 
                  
                 # Simulation handling
                 vision_sensor_handle=-1, # Coppelia's World handle 
 
                 # Intrinsic Parameters
                 resolution=(256,256), # Standard Coppelia resolution
                 fov_degrees=60, # Standard Coppelia fov
 
                 # Extrinsic Parameters
                 object_matrix=np.hstack((np.eye(3), np.zeros((3,1)))),
 
                 # Lens Distortion Model
                 distortion_model='rational',
                 distortion_coefficients=np.zeros(8),
 
                 # Image Noise Model
                 snr_dB=np.inf # No noise
                 ):
        
        # Simulation handling
        self.vision_sensor_handle = vision_sensor_handle

        # Intrinsic Parameters
        self.resolution = resolution
        self.aspect_ratio = resolution[0]/resolution[1]
        self.fov_degrees = fov_degrees

        # Extrinsic Parameters
        self.R, self.t = object_matrix[:, :-1], object_matrix[:, [-1]]
        
        # Compressing data into matrices
        self.intrinsic_matrix = build_intrinsic_matrix(fov_degrees, resolution)
        self.extrinsic_matrix = build_extrinsic_matrix(object_matrix)
        self.projection_matrix = build_projection_matrix(self.intrinsic_matrix, self.extrinsic_matrix)

        # Lens Distortion Model
        self.distortion_model = distortion_model
        self.distortion_coefficients = distortion_coefficients

        self.map_u, self.map_v = gen_distortion_maps(distortion_coefficients, distortion_model, self.intrinsic_matrix, resolution)

        # Image Noise Model
        self.snr_dB = snr_dB
        self.snr = np.power(10, (snr_dB / 20)) # Converted for ratio

    def get_coppelia_image(self, api_method):
        image_raw, resolution_x, resolution_y = api_method(self.vision_sensor_handle)

        image_unflipped = np.frombuffer(image_raw, dtype=np.uint8).reshape(resolution_y, resolution_x, 3) # 3 Channel image buffer 

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # This is consistent with the axes of vision sensors, pointing Z outwards, Y up
        image = cv2.flip(image_unflipped, 0)

        image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Use cv2.remap with the custom remapped coordinates
        image_distorted = cv2.remap(src=image_gray,
                                    dst=image_gray, 
                                    map1=self.map_u, 
                                    map2=self.map_v, 
                                    interpolation=cv2.INTER_NEAREST)
        
        if np.any(self.distortion_coefficients):
            kernel = np.ones((2,2),np.float32) / 4
            image_distorted = cv2.filter2D(image_distorted, -1 ,kernel)
        
        image_noisy = add_noise(image_distorted, self.snr)

        simulated_image = image_noisy

        return simulated_image