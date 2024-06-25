# Importing modules...
import numpy as np

from modules.vision.camera import *
from modules.vision.epipolar_geometry import * 

class MultipleView:
    def __init__(self, camera_models):
        # Camera model information
        self.camera_models = camera_models
        self.n_cameras = len(camera_models)

        # Fundamental matrices between each pair
        self.fundamental_matrix = np.array(np.zeros((self.n_cameras, self.n_cameras, 3, 3)))
        self.build_fundamental_matrices()

    def build_fundamental_matrices(self):
        # Build all fundamental matrices between camera pairs
        for reference in range(self.n_cameras):
            for auxiliary in range(self.n_cameras):
                if reference == auxiliary:
                    continue

                E = build_essential_matrix(self.camera_models[reference].extrinsic_matrix, self.camera_models[auxiliary].extrinsic_matrix)

                F = build_fundamental_matrix(self.camera_models[reference].intrinsic_matrix, self.camera_models[auxiliary].intrinsic_matrix, E)

                self.fundamental_matrix[reference][auxiliary] = F

    def triangulate_by_pair(self, pair, blobs_pair):
        reference, auxiliary = (0, 1) # Naming for the sake of code readability

        # Gathering pair info
        camera_pair = [self.camera_models[pair[reference]], self.camera_models[pair[auxiliary]]]
        pair_fundamental_matrix = self.fundamental_matrix[pair[reference]][pair[auxiliary]]
    
        # Compute epipolar lines
        epilines_auxiliary = cv2.computeCorrespondEpilines(points=blobs_pair[reference], 
                                                           whichImage=1, 
                                                           F=pair_fundamental_matrix).reshape(-1,3)
        
        # Order blobs
        blobs_pair[auxiliary] = epiline_order(blobs_pair[auxiliary], epilines_auxiliary)

        # Triangulate markers
        triangulated_points_4D = cv2.triangulatePoints(camera_pair[reference].projection_matrix.astype(np.float32), 
                                                       camera_pair[auxiliary].projection_matrix.astype(np.float32), 
                                                       blobs_pair[reference].T.astype(np.float32), 
                                                       blobs_pair[auxiliary].T.astype(np.float32))
        
        # Normalize homogeneous coordinates and discard last row
        triangulated_points_3D = (triangulated_points_4D / triangulated_points_4D[-1])[:-1, :]

        return triangulated_points_3D
    
    def calibrate(self, wand_blobs, wand_distances):
        # Camera pairs
        camera_ids = np.arange(self.n_cameras)
        reference = 0 # The 0th camera will be the reference by default
        pairs = [(reference, ID) for ID in camera_ids[camera_ids != reference]]

        # New extrinsic matrices
        self.camera_models[reference].update_reference(np.eye(4)[:3, :4]) 

        # Getting wand data
        wand_ratio = (1, wand_distances[1] / wand_distances[0]) 

        for pair in pairs:
            # Getting data from pair
            reference, auxiliary = pair
            
            # Synchronized blobs for each pair
            sync_blobs_pair = [wand_blobs[reference], wand_blobs[auxiliary]]
            
            # Order collinear blobs
            ordered_blobs_reference = []
            ordered_blobs_auxiliary = []

            for sync_blobs_reference, sync_blobs_auxiliary in zip(*sync_blobs_pair):
                blobs_reference = collinear_order(sync_blobs_reference, wand_ratio)
                blobs_auxiliary = collinear_order(sync_blobs_auxiliary, wand_ratio)

                if blobs_reference is None or blobs_auxiliary is None:
                    continue

                ordered_blobs_reference.append(blobs_reference)
                ordered_blobs_auxiliary.append(blobs_auxiliary)

            ordered_blobs_reference = np.array(ordered_blobs_reference)
            ordered_blobs_auxiliary = np.array(ordered_blobs_auxiliary)

            # Join all ordered blobs in a single matrix
            all_blobs_reference = np.vstack(ordered_blobs_reference)
            all_blobs_auxiliary = np.vstack(ordered_blobs_auxiliary)

            # Estimating and saving the Fundamental Matrix
            F_estimated, mask = cv2.findFundamentalMat(points1=all_blobs_reference, 
                                                       points2=all_blobs_auxiliary, 
                                                       method=cv2.FM_8POINT)
            
            self.fundamental_matrix[reference][auxiliary] = F_estimated

            # Selecting inlier points    
            inlier_all_blobs_reference = all_blobs_reference[mask.ravel() == 1]
            inlier_all_blobs_auxiliary = all_blobs_auxiliary[mask.ravel() == 1]

            mask = np.array([np.prod(flags) for flags in mask.reshape(-1, 3)])

            inlier_blobs_reference = ordered_blobs_reference[mask == 1]
            inlier_blobs_auxiliary = ordered_blobs_auxiliary[mask == 1]

            # Calculating essential matrix
            E = self.camera_models[auxiliary].intrinsic_matrix.T @ F_estimated @ self.camera_models[reference].intrinsic_matrix

            # Decomposing essential matrix
            R, t = decompose_essential_matrix(E,
                                              inlier_all_blobs_reference,
                                              inlier_all_blobs_auxiliary,
                                              self.camera_models[reference].intrinsic_matrix,
                                              self.camera_models[auxiliary].intrinsic_matrix)

            # Check if decomposition worked
            if R is None and t is None:
                print('> No valid decomposition!')
                continue

            object_matrix_auxiliary = np.hstack((R, t))
            extrinsic_matrix_auxiliary = np.linalg.inv(build_extrinsic_matrix(object_matrix_auxiliary))

            # Calculating projection matrices
            # The reference camera will be the reference frame, thus the identity matrix
            P_reference = build_projection_matrix(intrinsic_matrix=self.camera_models[reference].intrinsic_matrix, 
                                                  extrinsic_matrix=np.eye(4)) 
            
            P_auxiliary = build_projection_matrix(intrinsic_matrix=self.camera_models[auxiliary].intrinsic_matrix, 
                                                  extrinsic_matrix=extrinsic_matrix_auxiliary)
            
            all_triangulated_points_3D = [] # List of triangulated points
            all_unscaled_distances = [] # List of all measured distance
            scales = [] # Scale factor for each triangulated frame

            for points_reference_PTS, points_auxiliary_PTS in zip(inlier_blobs_reference, inlier_blobs_auxiliary):
                triangulated_points_4D = cv2.triangulatePoints(P_reference.astype(np.float32), 
                                                               P_auxiliary.astype(np.float32), 
                                                               points_reference_PTS.T.astype(np.float32), 
                                                               points_auxiliary_PTS.T.astype(np.float32))
                
                # Normalize homogeneous coordinates and discard last row
                triangulated_points_3D = (triangulated_points_4D / triangulated_points_4D[-1])[:-1, :]

                # Unscaled distances
                unscaled_distances = np.array([np.linalg.norm(triangulated_points_3D.T[0] - triangulated_points_3D.T[1]),
                                               np.linalg.norm(triangulated_points_3D.T[1] - triangulated_points_3D.T[2]),
                                               np.linalg.norm(triangulated_points_3D.T[0] - triangulated_points_3D.T[2])])

                # Save values
                all_triangulated_points_3D.append(triangulated_points_3D)
                all_unscaled_distances.append(unscaled_distances)
                scales.append(np.sum(wand_distances) / np.sum(unscaled_distances))

            # Mean scale factor
            scale = np.mean(np.array(scales))

            # Saving scaled matrix
            object_matrix_auxiliary = np.hstack((R, t * scale))
            extrinsic_matrix_auxiliary = np.linalg.inv(build_extrinsic_matrix(object_matrix_auxiliary))
            self.camera_models[auxiliary].update_reference(extrinsic_matrix_auxiliary[:3, :4])

        # Rebuild fundamental matrices with updated extrinsics
        self.build_fundamental_matrices()

def collinear_order(blobs, ratio):
    # Distances between blobs
    distances = np.array([np.linalg.norm(blobs[0] - blobs[1]), 
                          np.linalg.norm(blobs[1] - blobs[2]), 
                          np.linalg.norm(blobs[2] - blobs[0])])
    
    # Shortest distance
    shortest = np.min(distances)

    # If x is invalid
    if shortest == 0.0 or shortest is np.nan:
        return None

    # Normalize distances
    distances /= shortest

    # Measured unique distance sums
    measured_unique_sums = np.array([distances[0] + distances[2],
                                     distances[0] + distances[1],
                                     distances[1] + distances[2]])
    
    # Exprected unique distance sums
    expected_unique_sums = np.array([ratio[0] + ratio[0] + ratio[1],
                                     ratio[0] + ratio[1],
                                     ratio[0] + ratio[1] + ratio[1]])
    
    # Error matrix
    difference_matrix = np.array([[np.abs(measured - expected)
                                   for measured in measured_unique_sums] 
                                   for expected in expected_unique_sums])

    # Using the hungarian (Munkres) assignment algorithm to find unique correspondences between blobs and epilines
    _, new_indices = linear_sum_assignment(difference_matrix)

    return blobs[new_indices]