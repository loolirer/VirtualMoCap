# Importing modules...
import numpy as np

def build_intrinsic_matrix(fov_degrees, resolution):
    fov_radians = np.radians(fov_degrees)

    # Focal distances in pixels
    f_x = resolution[0]/(2*np.tan(fov_radians/2))
    f_y = resolution[1]/(2*np.tan(fov_radians/2))

    # Principle point
    o_x, o_y = resolution[0]/2, resolution[1]/2

    camera_matrix = np.array([[f_x,   0, o_x],
                              [  0, f_y, o_y],
                              [  0,   0,   1]])

    return camera_matrix

def build_extrinsic_matrix(object_matrix):
    if object_matrix.shape == (3,4): # Check if object matrix is in 3x4 format
        camera_pose = np.vstack((object_matrix, np.array([0, 0, 0, 1]))) # Make inversible 4x4 homogeneous transformation

    return camera_pose

def build_projection_matrix(intrinsic_matrix, extrinsic_matrix):
    if extrinsic_matrix.shape == (3,4): # Check if extrinsic matrix is in 3x4 format
        extrinsic_matrix = np.vstack((extrinsic_matrix, np.array([0, 0, 0, 1]))) # Make inversible 4x4 homogeneous transformation

    if intrinsic_matrix.shape == (3,3): # Check if intrinsic matrix is in 3x3 format
        intrinsic_matrix = np.hstack((intrinsic_matrix, np.zeros((3,1)))) # Convert to 3x4 shape

    projection_matrix = intrinsic_matrix @ np.linalg.inv(extrinsic_matrix) 

    return projection_matrix

def perspective_projection(points_to_project, projection_matrix):
    projected_points = projection_matrix @ points_to_project  # Project points to plane
    projected_points /= projected_points[-1]        # Normalize homogeneous coordinates
    projected_points = projected_points[:-1, :]     # Discard the last row
    projected_points = projected_points.astype(int) # Cast as interger
    
    return projected_points
