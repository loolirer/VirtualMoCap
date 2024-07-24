# Importing modules...
import numpy as np
import cv2

from modules.vision.linear_projection import *
from modules.vision.lens_distortion import *
from modules.vision.image_noise import *

class Camera:
    def __init__(self, 
                 resolution=(1, 1), 
                 
                 # Pinhole Camera Model
                 intrinsic_matrix=np.eye(3),
                 extrinsic_matrix=np.eye(4), 
 
                 # Lens Distortion Model
                 distortion_model=None, 
                 distortion_coefficients=np.zeros(4), 
 
                 # Image Noise Model
                 snr_dB=np.inf # No noise
                 ):

        # Image Parameters
        self.resolution = resolution
        self.aspect_ratio = self.resolution[0] / self.resolution[1]
        self.image_shape = self.resolution[::-1]

        # Pinhole Camera Model
        self.extrinsic_matrix = extrinsic_matrix
        self.intrinsic_matrix = intrinsic_matrix
        self.projection_matrix = build_projection_matrix(intrinsic_matrix=self.intrinsic_matrix, 
                                                         extrinsic_matrix=self.extrinsic_matrix)
        
        # Camera Pose
        self.pose = np.linalg.inv(self.extrinsic_matrix)

        # Lens Distortion Model
        self.distortion_model = distortion_model
        self.distortion_coefficients = distortion_coefficients

        # Distortion model parameters
        self.undistortion_map = None
        self.undistortion_function = None

        if self.distortion_model == 'rational' or self.distortion_model is None:
            self.undistortion_map = cv2.initUndistortRectifyMap
            self.undistortion_function = cv2.undistortPoints

        elif self.distortion_model == 'fisheye':
            self.undistortion_map = cv2.fisheye.initUndistortRectifyMap
            self.undistortion_function = cv2.fisheye.undistortPoints 

        self.map_u_d, self.map_v_d = build_distortion_map(self.distortion_coefficients, 
                                                          self.undistortion_map, 
                                                          self.intrinsic_matrix, 
                                                          self.resolution)
 
        # Image Noise Model
        self.snr_dB = snr_dB
        self.snr = np.power(10, (snr_dB / 20)) # Converted for ratio
    
    # Pinhole Camera Model Methods
    def update_intrinsic(self, new_intrinsic_matrix):
        self.intrinsic_matrix = new_intrinsic_matrix
        self.projection_matrix = build_projection_matrix(intrinsic_matrix=self.intrinsic_matrix, 
                                                         extrinsic_matrix=self.extrinsic_matrix)
        self.map_u_d, self.map_v_d = build_distortion_map(self.distortion_coefficients, 
                                                          self.undistortion_map, 
                                                          self.intrinsic_matrix, 
                                                          self.resolution)

    def update_extrinsic(self, new_extrinsic_matrix):
        self.extrinsic_matrix = new_extrinsic_matrix
        self.pose = np.linalg.inv(self.extrinsic_matrix)
        self.projection_matrix = build_projection_matrix(intrinsic_matrix=self.intrinsic_matrix, 
                                                         extrinsic_matrix=self.extrinsic_matrix)
    
    # Distortion Model Methods
    def undistort_points(self, distorted_points):
        return self.undistortion_function(distorted_points.reshape(1, -1, 2).astype(np.float32), 
                                          self.intrinsic_matrix, 
                                          self.distortion_coefficients,
                                          np.array([]),
                                          self.intrinsic_matrix).reshape(-1,2)
    
    def distort_image(self, image_pinhole):
        return cv2.remap(image_pinhole,
                         map1=self.map_u_d, 
                         map2=self.map_v_d, 
                         interpolation=cv2.INTER_NEAREST) 
    
    # Noise Model Methods
    def noise_image(self, image_noiseless):
        return add_noise(image_noiseless=image_noiseless, 
                         snr=self.snr)
