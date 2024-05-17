# Importing modules...
import numpy as np
from scipy.optimize import linear_sum_assignment

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