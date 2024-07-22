import numpy as np

# Camera matrices
intrinsic_matrix_0 = np.array([[720.313, 0, 481.014],
                               [0, 719.521, 360.991],
                               [0,       0,       1]])
distortion_coefficients_0 = np.array([0.395621,
                                      0.633705,
                                      -2.41723,
                                      2.11079], dtype=np.float32)

intrinsic_matrix_1 = np.array([[768.113, 0, 472.596],
                               [0, 767.935, 350.978],
                               [0,       0,       1]])
distortion_coefficients_1 = np.array([0.36891,
                                      1.50111,
                                      -7.9412,
                                      11.9171], dtype=np.float32)

intrinsic_matrix_2 =np.array([[728.237, 0, 459.854],
                              [0, 729.419, 351.590],
                              [0,       0,       1]])
distortion_coefficients_2 = np.array([0.27611,
                                      2.09465,
                                      -9.9795,
                                      14.1921], dtype=np.float32)

intrinsic_matrix_3 =np.array([[750.149, 0, 492.144],
                              [0, 748.903, 350.213],
                              [0,       0,       1]])
distortion_coefficients_3 = np.array([0.400774,
                                      1.159950,
                                      -7.10257,
                                      11.41500], dtype=np.float32)

# All in one
intrinsic_matrices = [intrinsic_matrix_0,
                      intrinsic_matrix_1,
                      intrinsic_matrix_2,
                      intrinsic_matrix_3]

distortion_coefficients = [distortion_coefficients_0,
                           distortion_coefficients_1,
                           distortion_coefficients_2,
                           distortion_coefficients_3]