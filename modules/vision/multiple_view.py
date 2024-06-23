# Importing modules...
import numpy as np

from modules.vision.camera import *
from modules.vision.epipolar_geometry import * 

class MultipleView:
    def __init__(self, camera_models):
        # Camera model information
        self.camera_models = camera_models
        self.n_cameras = len(camera_models)

        # Fundamental matrices between each pair
        self.fundamental_matrix = np.array(np.zeros((self.n_cameras, self.n_cameras, 3, 3)))
        self.build_fundamental_matrices()

    def build_fundamental_matrices(self):
        # Build all fundamental matrices between camera pairs
        for reference in range(self.n_cameras):
            for auxiliary in range(self.n_cameras):
                if reference == auxiliary:
                    continue

                E = build_essential_matrix(self.camera_models[reference].extrinsic_matrix, self.camera_models[auxiliary].extrinsic_matrix)

                F = build_fundamental_matrix(self.camera_models[reference].intrinsic_matrix, self.camera_models[auxiliary].intrinsic_matrix, E)

                self.fundamental_matrix[reference][auxiliary] = F

    def triangulate_by_pair(self, pair, blobs_pair):
        reference, auxiliary = (0, 1) # Naming for the sake of code readability

        # Gathering pair info
        camera_pair = [self.camera_models[pair[reference]], self.camera_models[pair[auxiliary]]]
        pair_fundamental_matrix = self.fundamental_matrix[pair[reference]][pair[auxiliary]]
    
        # Compute epipolar lines
        epilines_auxiliary = cv2.computeCorrespondEpilines(points=blobs_pair[reference], 
                                                           whichImage=1, 
                                                           F=pair_fundamental_matrix).reshape(-1,3)
        
        # Order blobs
        blobs_pair[auxiliary] = epiline_order(blobs_pair[auxiliary], epilines_auxiliary)

        # Triangulate markers
        triangulated_points_4D = cv2.triangulatePoints(camera_pair[reference].projection_matrix.astype(np.float32), 
                                                       camera_pair[auxiliary].projection_matrix.astype(np.float32), 
                                                       blobs_pair[reference].T.astype(np.float32), 
                                                       blobs_pair[auxiliary].T.astype(np.float32))
        
        # Normalize homogeneous coordinates and discard last row
        triangulated_points_3D = (triangulated_points_4D / triangulated_points_4D[-1])[:-1, :]

        return triangulated_points_3D