# Importing modules...
import numpy as np

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