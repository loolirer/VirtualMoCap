# Importing modules...
from scipy.interpolate import griddata
import numpy as np
import cv2

def distort_rational(image_point, intrinsic_matrix, rational_coefficients):
    # Get intrinsic parameters
    f_x, f_y = intrinsic_matrix[0][0], intrinsic_matrix[1][1]
    c_x, c_y = intrinsic_matrix[0][2], intrinsic_matrix[1][2]

    [[u], [v]] = image_point

    # Normalize coordinates
    x, y = (u - c_x)/f_x, (v - c_y)/f_y 

    normalized_image_point = np.array([[x],
                                       [y]])

    # Radial distance
    r = np.linalg.norm(normalized_image_point)

    # Get distortion coefficients (OpenCV's style)
    k1, k2, p1, p2, k3, k4, k5, k6 = rational_coefficients
    
    # Get radial and tangential transformation vectors
    radial_transformation = normalized_image_point * (1 + k1*r**2 + k2*r**4 + k3*r**6)/(1 + k4*r**2 + k5*r**4 + k6*r**6)

    tangential_transformation = np.array([[2*p1*x*y + p2*(r**2 + 2*x**2)],
                                          [p1*(r**2 + 2*y**2) + 2*p2*x*y]])
    
    # Get distorted normalized point
    [[x_d], [y_d]] = radial_transformation + tangential_transformation

    # Re-scale and re-center point
    u_d, v_d = x_d * f_x + c_x, y_d * f_y + c_y

    return np.array([[u_d],
                     [v_d]])

def distort_fisheye(image_point, intrinsic_matrix, fisheye_coefficients):
    # Get intrinsic parameters
    f_x, f_y = intrinsic_matrix[0][0], intrinsic_matrix[1][1]
    c_x, c_y = intrinsic_matrix[0][2], intrinsic_matrix[1][2]

    [[u], [v]] = image_point

    # Normalize coordinates
    x, y = (u - c_x)/f_x, (v - c_y)/f_y 

    normalized_image_point = np.array([[x],
                                       [y]])

    # Distortion parameters
    r = np.linalg.norm(normalized_image_point)
    theta = np.arctan(r)

    # Get distortion coefficients (OpenCV's style)
    k1, k2, k3, k4 = fisheye_coefficients

    if r != 0:
        [[x_d], [y_d]] = normalized_image_point * (theta + k1*theta**3 + k2*theta**5 + k3*theta**7 + k4*theta**9)/r
    else:
        [[x_d], [y_d]] = normalized_image_point # Do not distort

    # Re-scale and re-center point
    u_d, v_d = x_d * f_x + c_x, y_d * f_y + c_y

    return np.array([[u_d],
                     [v_d]])

def interpolate_map(map):
    missing_indices = np.array(np.where(np.isnan(map))).T

    # Create a grid of indices for the known values
    known_indices = np.array(np.where(~np.isnan(map))).T
    known_values = map[~np.isnan(map)]

    # Use interpolation to estimate missing values
    estimated_values = griddata(known_indices, known_values, missing_indices, method='nearest')

    # Replace missing values with the estimated values
    estimated_map = map.copy()
    estimated_map[np.isnan(map)] = estimated_values

    return estimated_map

def gen_distortion_maps(distortion_coefficients, model, intrinsic_matrix, resolution):
    # Make undistortion maps
    if model == 'rational':
        map_u, map_v = cv2.initUndistortRectifyMap(cameraMatrix=intrinsic_matrix,
                                                   distCoeffs=distortion_coefficients,
                                                   R=np.eye(3),
                                                   newCameraMatrix=intrinsic_matrix,
                                                   size=resolution,
                                                   m1type=cv2.CV_32FC1)
    
    elif model == 'fisheye':
        map_u, map_v = cv2.fisheye.initUndistortRectifyMap(K=intrinsic_matrix,
                                                           D=distortion_coefficients,
                                                           R=np.eye(3),
                                                           P=intrinsic_matrix,
                                                           size=resolution,
                                                           m1type=cv2.CV_32FC1)
        
    # Round maps and cast to integer for exact indexing 
    map_u = np.round(map_u).astype(int) 
    map_v = np.round(map_v).astype(int) 

    # Invert undistortion maps
    map_u_d = np.full(resolution, np.nan, dtype=np.float32)
    map_v_d = np.full(resolution, np.nan, dtype=np.float32)

    for v in range(resolution[0]):
        for u in range(resolution[1]):

            u_d = map_u[v][u] 
            v_d = map_v[v][u]

            # Do not remap points outside the image limits
            if (u_d >= 0 and u_d < resolution[1]) and (v_d >= 0 and v_d < resolution[0]):
                map_u_d[v_d][u_d] = u
                map_v_d[v_d][u_d] = v
    
    # Interpolate maps
    map_u_d = interpolate_map(map_u_d).astype(np.float32)
    map_v_d = interpolate_map(map_v_d).astype(np.float32)

    return map_u_d, map_v_d