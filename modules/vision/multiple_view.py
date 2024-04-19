# Importing modules...
import numpy as np

from modules.vision.camera import *
from modules.vision.epipolar_geometry import * 

class MultipleView:
    def __init__(self, camera):
        # Camera model information
        self.camera = camera
        self.n_cameras = len(camera)

        # Build all fundamental matrices between camera pairs
        self.fundamental_matrix = np.array(np.zeros((self.n_cameras, self.n_cameras, 3, 3)))

        for reference in range(self.n_cameras):
            for auxiliary in range(self.n_cameras):
                if reference == auxiliary:
                    continue

                E = build_essential_matrix(camera[reference].extrinsic_matrix, camera[auxiliary].extrinsic_matrix)

                F = build_fundamental_matrix(camera[reference].intrinsic_matrix, camera[auxiliary].intrinsic_matrix, E)

                self.fundamental_matrix[reference][auxiliary] = F

    def triangulate_by_pair(self, pair, distorted_blobs):
        reference, auxiliary = pair

        # Undistort blobs
        undistorted_blobs = [self.camera[ID].undistort_points(distorted_blobs[ID].T.reshape(1, -1, 2).astype(np.float32), 
                                                              self.camera[ID].intrinsic_matrix, 
                                                              self.camera[ID].distortion_coefficients,
                                                              np.array([]),
                                                              self.camera[ID].intrinsic_matrix).reshape(-1,2).T for ID in pair]

        # Compute epipolar lines
        epilines_auxiliary = cv2.computeCorrespondEpilines(points=undistorted_blobs[reference].T, 
                                                           whichImage=1, 
                                                           F=self.fundamental_matrix[reference][auxiliary]).reshape(-1,3)
        
        # Order blobs
        undistorted_blobs[auxiliary] = order_centroids(undistorted_blobs[auxiliary], epilines_auxiliary)

        # Triangulate markers
        triangulated_positions = cv2.triangulatePoints(self.camera[reference].projection_matrix.astype(float), 
                                                       self.camera[auxiliary].projection_matrix.astype(float), 
                                                       undistorted_blobs[reference].astype(float), 
                                                       undistorted_blobs[auxiliary].astype(float))
        
        # Normalize homogeneous coordinates
        triangulated_positions = (triangulated_positions / triangulated_positions[-1])[:-1, :]

        return triangulated_positions
        