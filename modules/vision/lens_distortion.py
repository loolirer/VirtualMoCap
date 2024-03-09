# Importing modules...
import numpy as np

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
                     [v_d]]).astype(int) # Cast as interger

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
                     [v_d]]).astype(int) # Cast as interger

def gen_distortion_maps(distortion_coefficients, model, intrinsic_matrix, resolution):
    map_u = np.zeros((resolution[0], resolution[1], 1), dtype=np.float32)
    map_v = np.zeros((resolution[0], resolution[1], 1), dtype=np.float32)

    distort = np.any(distortion_coefficients)

    # Note that the u-axis represents the columns and the v-axis represents the rows
    for v in range(resolution[0]):
        for u in range(resolution[1]):
            pixel_coordinate = np.array([[u],
                                         [v]])
            
            if distort: # If distortion parameters are available
                if model == 'rational':
                    distorted_pixel_coordinate = distort_rational(image_point=pixel_coordinate, 
                                                                  intrinsic_matrix=intrinsic_matrix,
                                                                  rational_coefficients=distortion_coefficients)
                elif model == 'fisheye':
                    distorted_pixel_coordinate = distort_fisheye(image_point=pixel_coordinate, 
                                                                 intrinsic_matrix=intrinsic_matrix,
                                                                 fisheye_coefficients=distortion_coefficients)
  
                [[u_d], [v_d]] = distorted_pixel_coordinate

                # Do not remap points outside the image limits
                if (u_d >= 0 and u_d < resolution[1]) and (v_d >= 0 and v_d < resolution[0]):
                    map_u[v_d][u_d] = u
                    map_v[v_d][u_d] = v

            else: # In case of no distortion 
                map_u[v][u] = u
                map_v[v][u] = v

    return map_u, map_v