# Importing modules...
import numpy as np

def build_intrinsic_matrix(fov_degrees, resolution):
    fov_radians = np.radians(fov_degrees) # Convert to radians

    # Focal distances in pixels
    f_x = resolution[0]/(2*np.tan(fov_radians/2))
    f_y = resolution[1]/(2*np.tan(fov_radians/2))

    aspect_ratio = resolution[0]/resolution[1]

    # Aspect ratio scaling
    if aspect_ratio > 1: # Landscape
        f_y *= aspect_ratio
    if aspect_ratio < 1: # Portrait
        f_x /= aspect_ratio

    # Principle point
    c_x, c_y = resolution[0]/2, resolution[1]/2

    intrinsic_matrix = np.array([[f_x,   0, c_x],
                                 [  0, f_y, c_y],
                                 [  0,   0,   1]])

    return intrinsic_matrix

def build_extrinsic_matrix(object_matrix):
    if object_matrix.shape == (3,4): # Check if object matrix is in 3x4 format
        extrinsic_matrix = np.vstack((object_matrix, np.array([0, 0, 0, 1]))) # Make inversible 4x4 homogeneous transformation
    elif object_matrix.shape == (4,4):
        extrinsic_matrix = object_matrix
    else:
        extrinsic_matrix = np.eye(4) 

    return extrinsic_matrix

def build_projection_matrix(intrinsic_matrix, extrinsic_matrix):
    if intrinsic_matrix.shape == (3,3): # Check if intrinsic matrix is in 3x3 format
        intrinsic_matrix = np.hstack((intrinsic_matrix, np.zeros((3,1)))) # Convert to 3x4 shape

    projection_matrix = intrinsic_matrix @ np.linalg.inv(extrinsic_matrix) 

    return projection_matrix

def perspective_projection(points_to_project, projection_matrix):
    points_to_project_h = np.vstack((points_to_project, np.ones(points_to_project.shape[1]))) # Convert to homogeneous
    projected_points = projection_matrix @ points_to_project_h  # Project points to plane
    projected_points /= projected_points[-1]        # Normalize homogeneous coordinates
    projected_points = projected_points[:-1, :]     # Discard the last row
    projected_points = projected_points.astype(int) # Cast as interger
    
    return projected_points
