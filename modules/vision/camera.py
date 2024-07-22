# Importing modules...
import numpy as np
import cv2

from modules.vision.blob_detection import *
from modules.vision.linear_projection import *
from modules.vision.lens_distortion import *
from modules.vision.image_noise import *

class Camera:
    def __init__(self, 
                 # Intrinsic Parameters
                 resolution, 
                 fov_degrees=None, # If not given, consider uncalibrated
                 intrinsic_matrix=np.eye(3),
 
                 # Camera Pose
                 pose=np.eye(4), 
 
                 # Lens Distortion Model
                 distortion_model=None,
                 distortion_coefficients=np.zeros(4),
 
                 # Image Noise Model
                 snr_dB=np.inf # No noise
                 ):

        # Intrinsic Parameters
        self.resolution = resolution
        self.aspect_ratio = resolution[0] / resolution[1]
        self.fov_degrees = None
        self.fov_radians = None

        # Camera Pose
        self.pose = pose
        
        # Compressing data into matrices
        self.extrinsic_matrix = np.linalg.inv(pose)
        self.intrinsic_matrix = intrinsic_matrix
        
        # If calibrated
        if fov_degrees is not None:
            self.fov_degrees = fov_degrees
            self.fov_radians = np.radians(fov_degrees)

            self.intrinsic_matrix = build_intrinsic_matrix(fov_degrees=self.fov_degrees, 
                                                           resolution=self.resolution)
            
        self.projection_matrix = build_projection_matrix(intrinsic_matrix=self.intrinsic_matrix, 
                                                         extrinsic_matrix=self.extrinsic_matrix)

        # Lens Distortion Model
        self.distortion_model = distortion_model
        self.distortion_coefficients = distortion_coefficients

        # Distortion model parameters
        self.undistortion_map = None
        self.undistortion_function = None
        self.map_u = None 
        self.map_v = None 

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
            
    def update_reference(self, new_camera_pose):
        # Update all extrinsic dependent parameters to new reference
        self.pose = new_camera_pose
        self.extrinsic_matrix = np.linalg.inv(self.pose)
        self.projection_matrix = build_projection_matrix(intrinsic_matrix=self.intrinsic_matrix, 
                                                         extrinsic_matrix=self.extrinsic_matrix)
        
    def undistort_points(self, distorted_centroids):
        return self.undistortion_function(distorted_centroids.reshape(1, -1, 2).astype(np.float32), 
                                          self.intrinsic_matrix, 
                                          self.distortion_coefficients,
                                          np.array([]),
                                          self.intrinsic_matrix).reshape(-1,2)