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
                 distortion_model=None,
                 distortion_coefficients=np.zeros(4),
 
                 # Image Noise Model
                 snr_dB=np.inf # No noise
                 ):
        
        # Simulation handling
        self.vision_sensor_handle = vision_sensor_handle

        # Intrinsic Parameters
        self.resolution = resolution
        self.aspect_ratio = resolution[0]/resolution[1]
        self.fov_degrees = fov_degrees
        self.fov_radians = np.radians(fov_degrees)

        # Extrinsic Parameters
        self.object_matrix = object_matrix
        self.R, self.t = self.object_matrix[:, :-1], self.object_matrix[:, [-1]]
        
        # Compressing data into matrices
        self.intrinsic_matrix = build_intrinsic_matrix(fov_degrees=self.fov_degrees, 
                                                       resolution=self.resolution)
        self.extrinsic_matrix = build_extrinsic_matrix(object_matrix=self.object_matrix)
        self.projection_matrix = build_projection_matrix(intrinsic_matrix=self.intrinsic_matrix, 
                                                         extrinsic_matrix=self.extrinsic_matrix)

        # Lens Distortion Model
        self.distortion_model = distortion_model
        self.distortion_coefficients = distortion_coefficients

        # Build distortion model parameters
        self.undistortion_map = None
        self.undistortion_function = None

        if self.distortion_model == 'rational' or self.distortion_model is None:
            self.undistortion_map = cv2.initUndistortRectifyMap
            self.undistortion_function = cv2.undistortPoints

        elif self.distortion_model == 'fisheye':
            self.undistortion_map = cv2.fisheye.initUndistortRectifyMap
            self.undistortion_function = cv2.fisheye.undistortPoints

        self.map_u, self.map_v = build_distortion_map(self.distortion_coefficients, 
                                                      self.undistortion_map, 
                                                      self.intrinsic_matrix, 
                                                      self.resolution)

        # Image Noise Model
        self.snr_dB = snr_dB
        self.snr = np.power(10, (snr_dB / 20)) # Converted for ratio

    def get_coppelia_image(self, api_method):
        # Get grayscale image buffer
        buffer, resolution = api_method(self.vision_sensor_handle, 1) # Set second argument to 1 for grayscale, 0 for RGB

        # Convert buffer into single channel image
        image_unflipped = np.frombuffer(buffer, dtype=np.uint8).reshape(resolution[1], resolution[0])

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # This is consistent with the axes of vision sensors, pointing Z outwards, Y up
        image_gray = cv2.flip(image_unflipped, 0)

        # Use cv2.remap with the custom remapped coordinates
        image_distorted = cv2.remap(src=image_gray,
                                    dst=image_gray, 
                                    map1=self.map_u, 
                                    map2=self.map_v, 
                                    interpolation=cv2.INTER_NEAREST)
        
        # Add noise based on the desired SNR for the image
        image_noisy = add_noise(image_distorted, self.snr)

        simulated_image = image_noisy

        return simulated_image
    
    def undistort_points(self, distorted_centroids):
        return self.undistortion_function(distorted_centroids.reshape(1, -1, 2).astype(np.float32), 
                                          self.intrinsic_matrix, 
                                          self.distortion_coefficients,
                                          np.array([]),
                                          self.intrinsic_matrix).reshape(-1,2)