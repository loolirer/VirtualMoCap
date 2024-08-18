import numpy as np
from scipy.optimize import linear_sum_assignment

def perpendicular_order(markers, wand_distances):
    # Distances between markers
    distances = np.array([np.linalg.norm(markers[0] - markers[1]), 
                          np.linalg.norm(markers[1] - markers[2]), 
                          np.linalg.norm(markers[2] - markers[0])])

    # Measured unique distance sums
    measured_unique_sums = np.array([distances[0] + distances[2],
                                     distances[0] + distances[1],
                                     distances[1] + distances[2]])
    
    # Expected unique distance sums
    expected_unique_sums = np.array([wand_distances[0] + wand_distances[1],
                                     wand_distances[0] + np.sqrt(wand_distances[0]**2 + wand_distances[1]**2),
                                     wand_distances[1] + np.sqrt(wand_distances[0]**2 + wand_distances[1]**2)])
    
    # Error matrix
    difference_matrix = np.array([[np.abs(measured - expected)
                                   for measured in measured_unique_sums] 
                                   for expected in expected_unique_sums])

    # Using the hungarian (Munkres) assignment algorithm to find unique correspondences between blobs and epilines
    _, new_indices = linear_sum_assignment(difference_matrix)

    return markers[new_indices]

# Find transformation to align point-set 'align' to point-set 'fixed'
def kabsch(align, fixed):
    assert align.shape == fixed.shape

    # Find centroids and ensure they are 3x1
    align_c = np.mean(align, axis=1).reshape(-1, 1)
    fixed_c = np.mean(fixed, axis=1).reshape(-1, 1)

    # Centralize point-sets in origin
    align_0 = align - align_c
    fixed_0 = fixed - fixed_c

    H = align_0 @ np.transpose(fixed_0)

    # Find rotation using Singular Value Decomposition
    try:
        U, _, Vt = np.linalg.svd(H)

    except:
        return np.full((4, 4), np.nan) # SVD did not converge

    R = Vt.T @ U.T

    # Special reflection case
    if np.linalg.det(R) < 0: # The rotation matrix determinant should be +1
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    # Finding translation vector
    t = -R @ align_c + fixed_c

    return np.vstack((np.hstack((R, t)), np.array([0, 0, 0, 1])))