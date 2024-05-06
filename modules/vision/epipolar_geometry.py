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

def point_to_line_distance(point, line):
    a, b, c = line
    x, y = point

    distance = np.abs(a * x + b * y + c) / np.sqrt(a**2 + b**2)

    return distance

def epiline_order(blobs_auxiliary, epilines_auxiliary):
    # Ordered centroids based on reference's centroids 
    ordered_blobs_auxiliary = np.zeros_like(blobs_auxiliary)

    for blob_auxiliary in blobs_auxiliary:
        distances = [] # Distance from the centroid to each epipolar line

        for epiline_auxiliary in epilines_auxiliary:
            distances.append(point_to_line_distance(blob_auxiliary, epiline_auxiliary)) # Distance from the centroid to an epipolar line

        new_index = np.argmin(np.array(distances)) # The match will be made for the shortest point to line distance

        ordered_blobs_auxiliary[new_index] = blob_auxiliary # Assigning the new centroid order

    return ordered_blobs_auxiliary