# Importing modules...
import numpy as np

from modules.vision.camera import *
from modules.vision.epipolar_geometry import * 

class MultipleView:
    def __init__(self, cameras):
        # Camera model information
        self.cameras = cameras
        self.n_cameras = len(cameras)

        # Build all fundamental matrices between camera pairs
        self.fundamental_matrix = np.array(np.zeros((self.n_cameras, self.n_cameras, 3, 3)))

        for reference in range(self.n_cameras):
            for auxiliary in range(self.n_cameras):
                if reference == auxiliary:
                    continue

                E = build_essential_matrix(self.cameras[reference].extrinsic_matrix, self.cameras[auxiliary].extrinsic_matrix)

                F = build_fundamental_matrix(self.cameras[reference].intrinsic_matrix, self.cameras[auxiliary].intrinsic_matrix, E)

                self.fundamental_matrix[reference][auxiliary] = F

    def triangulate_by_pair(self, pair, distorted_centroids_pair):
        # Gathering pair info
        camera_pair = [self.cameras[pair[0]], self.cameras[pair[1]]]
        pair_fundamental_matrix = self.fundamental_matrix[pair[0]][pair[1]]
        
        reference, auxiliary = (0, 1) # Naming for the sake of code readability

        # Undistort blobs
        undistorted_centroids_pair = [camera.undistort_points(distorted_centroids.T.reshape(1, -1, 2).astype(np.float32), 
                                                              camera.intrinsic_matrix, 
                                                              camera.distortion_coefficients,
                                                              np.array([]),
                                                              camera.intrinsic_matrix).reshape(-1,2).T 
                                                                           
                                      for (camera, distorted_centroids) in zip(camera_pair, distorted_centroids_pair)]

        # Compute epipolar lines
        epilines_auxiliary = cv2.computeCorrespondEpilines(points=undistorted_centroids_pair[reference].T, 
                                                           whichImage=1, 
                                                           F=pair_fundamental_matrix).reshape(-1,3)
        
        # Order blobs
        undistorted_centroids_pair[auxiliary] = order_centroids(undistorted_centroids_pair[auxiliary], epilines_auxiliary)

        # Triangulate markers
        triangulated_points_4D = cv2.triangulatePoints(camera_pair[reference].projection_matrix.astype(float), 
                                                       camera_pair[auxiliary].projection_matrix.astype(float), 
                                                       undistorted_centroids_pair[reference].astype(float), 
                                                       undistorted_centroids_pair[auxiliary].astype(float))
        
        # Normalize homogeneous coordinates and discard last row
        triangulated_points_3D = (triangulated_points_4D / triangulated_points_4D[-1])[:-1, :]

        return triangulated_points_3D
        