# Importing modules...
import numpy as np
import cv2
from scipy.optimize import linear_sum_assignment

from modules.vision.linear_projection import *

def build_essential_matrix(extrinsic_matrix_reference, extrinsic_matrix_auxiliary):
    # Compute relative transformation between the camera pair
    relative_transformation = np.linalg.inv(extrinsic_matrix_auxiliary) @ extrinsic_matrix_reference
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

def point_to_line_distance(point, line):
    a, b, c = line
    x, y = point

    distance = np.abs(a * x + b * y + c) / np.sqrt(a**2 + b**2)

    return distance

def epiline_order(blobs_auxiliary, epilines_auxiliary):
    distance_matrix = np.array([[point_to_line_distance(blob_auxiliary, epiline_auxiliary)
                                for blob_auxiliary in blobs_auxiliary] 
                                for epiline_auxiliary in epilines_auxiliary])

    # Using the hungarian (Munkres) assignment algorithm to find unique correspondences between blobs and epilines
    _, new_indices = linear_sum_assignment(distance_matrix)

    return blobs_auxiliary[new_indices]

def decompose_essential_matrix(E, 
                               blobs_reference, 
                               blobs_auxiliary, 
                               intrinsic_matrix_reference, 
                               intrinsic_matrix_auxiliary):
    
    # Decompose the essential matrix
    R1, R2, t = cv2.decomposeEssentialMat(E)
    
    # Possible rotations and translations for the auxiliary camera
    R_t_options = [[R1,  t],
                   [R1, -t],
                   [R2,  t],
                   [R2, -t]]
    
    best_option = None

    # Projection matrix of the first camera
    P_reference = build_projection_matrix(intrinsic_matrix=intrinsic_matrix_reference,
                                          extrinsic_matrix=np.eye(4))
    
    # Triangulate points and check their validity
    for option, R_t in enumerate(R_t_options):
        P_auxiliary = build_projection_matrix(intrinsic_matrix=intrinsic_matrix_auxiliary,
                                              extrinsic_matrix=build_extrinsic_matrix(np.hstack(R_t)))

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
            best_option = option # Where all points are frontal points

    # No valid decomposition
    if best_option is None:
        return None, None 
    
    # Valid decomposition
    return R_t_options[best_option]