import numpy as np

# Calibration data
intrinsic_matrix_0 = [
    [720.313, 0, 481.014],
    [0, 719.521, 360.991],
    [0,       0,       1]
]

distortion_coeffients_0 = [0.395621, 0.633705, -2.41723, 2.11079]

intrinsic_matrix_1 = [
    [768.113, 0, 472.596],
    [0, 767.935, 350.978],
    [0,       0,       1]
]

distortion_coefficients_1 = [0.368917, 1.50111, -7.94126, 11.9171]

intrinsic_matrix_2 = [
    [728.237, 0, 459.854],
    [0, 729.419, 351.590],
    [0,       0,       1]
]

distortion_coefficients_2 = [0.276114, 2.09465, -9.97956, 14.1921]

intrinsic_matrix_3 =[
    [750.149, 0, 492.144],
    [0, 748.903, 350.213],
    [0,       0,       1]     
]

distortion_coefficients_3 = [0.400774, 1.15995, -7.10257, 11.4150]

# Join data
all_intrinsic_matrices = np.array([
    intrinsic_matrix_0, 
    intrinsic_matrix_1, 
    intrinsic_matrix_2, 
    intrinsic_matrix_3
])

all_distortion_coefficients = np.array([
    distortion_coeffients_0, 
    distortion_coefficients_1, 
    distortion_coefficients_2, 
    distortion_coefficients_3
]).astype(np.float32)