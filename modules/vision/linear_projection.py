# Importing modules...
import numpy as np

def build_intrinsic_matrix(fov_degrees, resolution):
    fov_radians = np.radians(fov_degrees) # Convert to radians

    # Focal distances in pixels
    f_x = resolution[0] / (2 * np.tan(fov_radians / 2))
    f_y = resolution[1] / (2 * np.tan(fov_radians / 2))

    aspect_ratio = resolution[0] / resolution[1]

    # Aspect ratio scaling
    if aspect_ratio > 1: # Landscape
        f_y *= aspect_ratio
    if aspect_ratio < 1: # Portrait
        f_x /= aspect_ratio

    # Principle point
    c_x, c_y = resolution[0] / 2, resolution[1] / 2

    intrinsic_matrix = np.array([[f_x,   0, c_x],
                                 [  0, f_y, c_y],
                                 [  0,   0,   1]])

    return intrinsic_matrix

def break_intrinsic_matrix(intrinsic_matrix):
    f = (intrinsic_matrix[0][0] + intrinsic_matrix[1][1]) / 2 # Get mean focal distance in pixels
    resolution = (round(intrinsic_matrix[0][2] * 2), round(intrinsic_matrix[1][2] * 2)) # Extract resolution in pixels
    fov_radians = 2 * np.arctan(max(resolution) / (2 * f)) # Get FOV from focal distance and resolution
    fov_degrees = np.degrees(fov_radians) # Convert to degrees

    return resolution, fov_degrees

def build_projection_matrix(intrinsic_matrix, extrinsic_matrix):
    projection_matrix = np.hstack((intrinsic_matrix, np.zeros((3, 1)))) @ extrinsic_matrix 

    return projection_matrix

def perspective_projection(points_to_project, projection_matrix):
    points_to_project_h = np.vstack((points_to_project, np.ones(points_to_project.shape[1]))) # Convert to homogeneous
    projected_points = projection_matrix @ points_to_project_h # Project points to plane
    projected_points /= projected_points[-1] # Normalize homogeneous coordinates
    projected_points = projected_points[:-1, :] # Discard the last row
    
    return projected_points.T

def reprojection_error(points_to_project, points_in_image, projection_matrix):
    projected_points = perspective_projection(points_to_project, projection_matrix)
    residuals = np.linalg.norm(points_in_image - projected_points, axis=1)

    return residuals
