# Importing modules...
import numpy as np
import cv2
from scipy.optimize import linear_sum_assignment

from modules.vision.linear_projection import *

def build_essential_matrix(extrinsic_matrix_reference, extrinsic_matrix_auxiliary):
    # Compute relative transformation between the camera pair
    relative_transformation = extrinsic_matrix_auxiliary @ np.linalg.inv(extrinsic_matrix_reference)
    R, t = relative_transformation[0:3, 0:3], relative_transformation[0:3 , [-1]]

    t = t.ravel() # Make array unidimensional
    t_ss = np.array([[    0, -t[2],  t[1]],
                     [ t[2],     0, -t[0]],
                     [-t[1],  t[0],     0]]) # Skew symmetric matrix

    essential_matrix = t_ss @ R

    return essential_matrix

def build_fundamental_matrix(intrinsic_matrix_reference, intrinsic_matrix_auxiliary, essential_matrix):
    fundamental_matrix = np.linalg.inv(intrinsic_matrix_auxiliary).T @ essential_matrix @ np.linalg.inv(intrinsic_matrix_reference)

    return fundamental_matrix

def epiline_order(blobs_reference, blobs_auxiliary, fundamental_matrix):
    # Computing epilines
    epilines_auxiliary = cv2.computeCorrespondEpilines(points=blobs_reference, 
                                                       whichImage=1, 
                                                       F=fundamental_matrix).reshape(-1,3)

    blobs_auxiliary_h = np.hstack((blobs_auxiliary, np.ones(blobs_auxiliary.shape[0]).reshape(-1, 1)))

    # Point to line distance matrix
    distance_matrix = np.abs([[blob_auxiliary @ epiline_auxiliary 
                               for blob_auxiliary in blobs_auxiliary_h] 
                               for epiline_auxiliary in epilines_auxiliary])

    # Check for ambiguities
    line_mapping = np.argmin(distance_matrix, axis=1)
    unique_mapping = np.unique(line_mapping)

    # Blobs to epiline correspondences are unique
    if line_mapping.shape == unique_mapping.shape:
        return blobs_auxiliary[line_mapping]

    # Blobs to epiline correspondences are ambiguous
    return np.full_like(blobs_auxiliary, np.nan)
    
def decompose_essential_matrix(E, 
                               blobs_reference, 
                               blobs_auxiliary, 
                               intrinsic_matrix_reference, 
                               intrinsic_matrix_auxiliary):
    
    # Decompose the essential matrix
    R1, R2, t = cv2.decomposeEssentialMat(E)
    R_nan, t_nan = np.full_like(np.eye(3), np.nan), np.full_like(t, np.nan)
    
    # Possible transformations from the reference frame to the auxiliary frame
    R_t_options = [[R1,  t],
                   [R1, -t],
                   [R2,  t],
                   [R2, -t]]
    
    best_option = None

    # Projection matrix of the first camera
    P_reference = intrinsic_matrix_reference @ np.eye(4)[:3, :4]
    
    # Triangulate points and check their validity
    for option, R_t in enumerate(R_t_options):
        P_auxiliary = intrinsic_matrix_auxiliary @ np.hstack(R_t)

        triangulated_points_4D = cv2.triangulatePoints(P_reference.astype(np.float32), 
                                                       P_auxiliary.astype(np.float32), 
                                                       blobs_reference.T.astype(np.float32), 
                                                       blobs_auxiliary.T.astype(np.float32))
        
        # Normalize homogeneous coordinates and discard last row
        triangulated_points_3D = (triangulated_points_4D / triangulated_points_4D[-1])[:-1, :]

        # Count how many points have a positive z-coordinate 
        # This indicates if the point is in front of the camera or not
        frontal_points = np.count_nonzero(triangulated_points_3D[2] > 0)

        # No markers were shot behind the cameras
        if frontal_points == triangulated_points_3D.shape[1]:
            # Ambiguous decompostions found
            if best_option is not None:
                return R_nan, t_nan
            
            best_option = option # Where all points are frontal points

    # No valid decomposition found
    if best_option is None:
        return R_nan, t_nan
    
    # Valid decomposition
    return R_t_options[best_option]