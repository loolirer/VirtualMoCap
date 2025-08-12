# Importing modules...
import numpy as np
import scipy as sp

from virtualmocap.vision.camera import *
from virtualmocap.vision.epipolar_geometry import *
from virtualmocap.vision.rigid_transformations import *


class MultipleView:
    def __init__(self, camera_models):
        # Camera model information
        self.camera_models = camera_models
        self.n_cameras = len(camera_models)

        # Fundamental matrices between each pair
        self.build_fundamental_matrices()

    def build_fundamental_matrices(self):
        # Build all fundamental matrices between camera pairs
        self.fundamental_matrix = np.array(
            np.zeros((self.n_cameras, self.n_cameras, 3, 3))
        )

        for reference in range(self.n_cameras):
            for auxiliary in range(self.n_cameras):
                if reference == auxiliary:
                    continue

                E = build_essential_matrix(
                    self.camera_models[reference].extrinsic_matrix,
                    self.camera_models[auxiliary].extrinsic_matrix,
                )

                F = build_fundamental_matrix(
                    self.camera_models[reference].intrinsic_matrix,
                    self.camera_models[auxiliary].intrinsic_matrix,
                    E,
                )

                self.fundamental_matrix[reference][auxiliary] = F

    def stereo_triangulation_logic(self, pair, blobs_pair, order=True):
        reference, auxiliary = (0, 1)  # Naming for the sake of code readability

        # Gathering pair info
        camera_pair = [
            self.camera_models[pair[reference]],
            self.camera_models[pair[auxiliary]],
        ]
        pair_fundamental_matrix = self.fundamental_matrix[pair[reference]][
            pair[auxiliary]
        ]

        # Order blobs if needed
        if order:
            blobs_pair[auxiliary] = epiline_order(
                blobs_pair[reference], blobs_pair[auxiliary], pair_fundamental_matrix
            )

        # Ambiguous epiline ordering, discard data
        if np.isnan(blobs_pair[auxiliary]).any():
            return np.full((3, blobs_pair[auxiliary].shape[0]), np.nan)

        # Triangulate markers
        triangulated_points_4D = cv2.triangulatePoints(
            camera_pair[reference].projection_matrix.astype(np.float32),
            camera_pair[auxiliary].projection_matrix.astype(np.float32),
            blobs_pair[reference].T.astype(np.float32),
            blobs_pair[auxiliary].T.astype(np.float32),
        )

        # Normalize homogeneous coordinates and discard last row
        triangulated_points_3D = (triangulated_points_4D / triangulated_points_4D[-1])[
            :-1, :
        ]

        return triangulated_points_3D

    # This can re-triangulate the same point
    # This doesn't account any ordering method
    def multivision_triangulation_logic(
        self,
        points_in_images,
        reprojection_tol=1,
        collinearity_tol=0.005,
        min_views=2,
    ):
        # Camera identifiers
        camera_ids = np.arange(self.n_cameras)
        min_views = max(min_views, 2)  # Only 2 views or more

        # Sort cameras and their respective image points from highest number of detected markers to lowest
        camera_ids, camera_models, points_in_images = zip(
            *sorted(
                zip(camera_ids, self.camera_models, points_in_images),
                key=lambda x: len(x[-1]),
                reverse=True,
            )
        )
        views = [view for view in zip(camera_ids, camera_models, points_in_images)]

        # Iterate through each view, going from highest number of detected markers to lowest
        triangulated_points = []
        for v, (ref, camera_ref, points_in_image) in enumerate(views):
            # In a view, search each point
            for point_in_image_ref in points_in_image:
                # Save each unique complimentary view here
                triangulation_buffer = [
                    [camera_ref.projection_matrix, point_in_image_ref]
                ]

                # Updated views
                updated_views = []

                # For the current point, get all views in which a correspondence
                # can be made in a non-ambiguous way
                for aux, camera_aux, points_in_image_aux in views[v + 1 :]:
                    # If the other view has no points, just skip
                    if not len(points_in_image_aux):
                        continue

                    # Get matches of the reference point in the auxiliar view
                    matches = get_other_view(
                        point_in_image_ref,
                        points_in_image_aux,
                        self.fundamental_matrix[ref][aux],
                        collinearity_tol,
                    )

                    # Other views of the point in reference image
                    other_views = points_in_image_aux[matches]

                    # Check if the point has only one correspondence (not ambiguous)
                    if len(other_views) == 1:
                        other_view = other_views[0]

                        triangulation_buffer.append(
                            [camera_aux.projection_matrix, other_view]
                        )

                        points_in_image_aux = np.delete(
                            points_in_image_aux, matches, axis=0
                        )

                    # Generate possible updated view
                    updated_views.append((aux, camera_aux, points_in_image_aux))

                # Do not try to triangulate if only one view is available
                if len(triangulation_buffer) >= min_views:
                    # Triangulate a marker with multiple views
                    triangulated_point = triangulate_by_multivision(
                        *zip(*triangulation_buffer)
                    )

                    # If maximum reprojection error is within the tolerance
                    if (
                        max_reprojection_error(
                            triangulated_point, *zip(*triangulation_buffer)
                        )
                        < reprojection_tol
                    ):
                        # Save triangulated points
                        triangulated_points.append(triangulated_point)

                        # If triangulation was made with 3 or more views, update views
                        if len(triangulation_buffer) >= 3:
                            views[v + 1 :] = updated_views

            # Continue the same process in another camera

        # If any point was triangulated
        if triangulated_points:
            return np.array(triangulated_points).T

        # No point triangulated
        return np.full((3, 1), np.nan)

    def calibrate(self, wand_blobs, wand_distances):
        # Getting wand data
        wand_ratio = (1.0, wand_distances[1] / wand_distances[0])

        # Camera pairs
        camera_ids = np.arange(self.n_cameras)
        pairs = [(i, j) for i in camera_ids for j in camera_ids[camera_ids != i]]

        # Dicts for storing the final data
        relative_poses = {}
        extrinsic_matrices = {}
        triangulated_markers = {}
        possible_references = [True for _ in camera_ids]

        # Order collinear blobs
        all_ordered_blobs_per_frame = []
        for same_frame_blobs in zip(*wand_blobs):
            ordered_blobs = [
                collinear_order(same_camera_blobs, wand_ratio)
                for same_camera_blobs in same_frame_blobs
            ]

            # Only accept blobs valid in all views
            if np.isnan(ordered_blobs).any():
                continue

            all_ordered_blobs_per_frame.append(ordered_blobs)
        all_ordered_blobs_per_frame = np.array(all_ordered_blobs_per_frame)

        for pair in pairs:
            # Getting data from pair
            reference, auxiliary = pair

            if not possible_references[reference]:
                continue

            # Synchronized blobs for each camera in pair
            ordered_blobs_reference_per_frame = all_ordered_blobs_per_frame[
                :, reference, :, :
            ]
            ordered_blobs_auxiliary_per_frame = all_ordered_blobs_per_frame[
                :, auxiliary, :, :
            ]

            # Join all ordered blobs in a single matrix
            all_blobs_reference = np.vstack(ordered_blobs_reference_per_frame)
            all_blobs_auxiliary = np.vstack(ordered_blobs_auxiliary_per_frame)

            # Estimating and saving the Fundamental Matrix
            F_estimated, mask = cv2.findFundamentalMat(
                points1=all_blobs_reference,
                points2=all_blobs_auxiliary,
                method=cv2.FM_8POINT,
            )

            # Selecting inlier points
            inlier_all_blobs_reference = all_blobs_reference[mask.ravel() == 1]
            inlier_all_blobs_auxiliary = all_blobs_auxiliary[mask.ravel() == 1]

            mask = np.array([np.prod(flags) for flags in mask.reshape(-1, 3)])

            inlier_blobs_reference_per_frame = ordered_blobs_reference_per_frame[
                mask == 1
            ]
            inlier_blobs_auxiliary_per_frame = ordered_blobs_auxiliary_per_frame[
                mask == 1
            ]

            # Calculating essential matrix
            E = (
                self.camera_models[auxiliary].intrinsic_matrix.T
                @ F_estimated
                @ self.camera_models[reference].intrinsic_matrix
            )

            # Decomposing essential matrix
            R, t = decompose_essential_matrix(
                E,
                inlier_all_blobs_reference,
                inlier_all_blobs_auxiliary,
                self.camera_models[reference].intrinsic_matrix,
                self.camera_models[auxiliary].intrinsic_matrix,
            )

            # Check if decomposition worked
            if np.isnan(R).any() and np.isnan(t).any():
                possible_references[reference] = False

            # Calculating projection matrices
            # The reference camera will be the reference frame, thus the identity matrix
            P_reference = (
                self.camera_models[reference].intrinsic_matrix @ np.eye(4)[:3, :4]
            )
            P_auxiliary = self.camera_models[auxiliary].intrinsic_matrix @ np.hstack(
                (R, t)
            )

            all_triangulated_points = []  # List of triangulated points
            all_unscaled_distances = []  # List of all measured distance
            scales = []  # Scale factor for each triangulated frame

            for points_reference_per_frame, points_auxiliary_per_frame in zip(
                inlier_blobs_reference_per_frame, inlier_blobs_auxiliary_per_frame
            ):
                triangulated_points_h = cv2.triangulatePoints(
                    P_reference.astype(np.float32),
                    P_auxiliary.astype(np.float32),
                    points_reference_per_frame.T.astype(np.float32),
                    points_auxiliary_per_frame.T.astype(np.float32),
                )

                # Normalize homogeneous coordinates and discard last row
                triangulated_points = (
                    triangulated_points_h / triangulated_points_h[-1]
                )[:-1, :]

                # Unscaled distances
                unscaled_distances = np.array(
                    [
                        np.linalg.norm(
                            triangulated_points.T[0] - triangulated_points.T[1]
                        ),
                        np.linalg.norm(
                            triangulated_points.T[1] - triangulated_points.T[2]
                        ),
                        np.linalg.norm(
                            triangulated_points.T[0] - triangulated_points.T[2]
                        ),
                    ]
                )

                # Save values
                all_triangulated_points.append(triangulated_points)
                all_unscaled_distances.append(unscaled_distances)
                scales.append(np.sum(wand_distances) / np.sum(unscaled_distances))

            # Mean scale factor
            scale = np.mean(np.array(scales))

            scaled_triangulated_points = (
                np.hstack(np.array(all_triangulated_points)) * scale
            )
            triangulated_markers[pair] = scaled_triangulated_points

            # Saving scaled matrices
            extrinsic_matrix_auxiliary = np.vstack(
                (np.hstack((R, t * scale)), np.array([0, 0, 0, 1]))
            )
            extrinsic_matrices[pair] = extrinsic_matrix_auxiliary
            relative_poses[pair] = np.linalg.inv(extrinsic_matrix_auxiliary)

        # Update references
        available_references = np.where(possible_references)[0].tolist()

        if not available_references:
            return False  # Calibration failed!

        reference = available_references[0]  # Get the first available reference
        self.camera_models[reference].update_extrinsic(np.eye(4))
        for ID in camera_ids[camera_ids != reference]:
            self.camera_models[ID].update_extrinsic(extrinsic_matrices[(reference, ID)])

        # Rebuild fundamental matrices with updated references
        self.build_fundamental_matrices()

        return True  # Calibration succeeded!

    def bundle_adjustment(self, wand_blobs, wand_distances, n_observations):
        # Getting wand data
        wand_ratio = (1.0, wand_distances[1] / wand_distances[0])

        # Order collinear blobs
        all_ordered_blobs_per_frame = []
        for same_frame_blobs in zip(*wand_blobs):
            ordered_blobs = [
                collinear_order(same_camera_blobs, wand_ratio)
                for same_camera_blobs in same_frame_blobs
            ]

            # Only accept blobs valid in all views
            if np.isnan(ordered_blobs).any():
                continue

            all_ordered_blobs_per_frame.append(ordered_blobs)
        all_ordered_blobs_per_frame = np.array(all_ordered_blobs_per_frame)

        # All pairs with no repetition
        camera_ids = np.arange(self.n_cameras)
        unique_pairs = [(i, j) for i in camera_ids for j in camera_ids[i + 1 :]]

        # Triangulate points for each pair
        all_triangulated_points = []
        for pair in unique_pairs:
            reference, auxiliary = pair

            triangulated_points = []
            for blobs_in_frame in all_ordered_blobs_per_frame:
                triangulated_points.append(
                    self.stereo_triangulation_logic(
                        pair,
                        [blobs_in_frame[reference], blobs_in_frame[auxiliary]],
                        order=False,
                    )
                )

            triangulated_points = np.hstack(np.array(triangulated_points))
            all_triangulated_points.append(triangulated_points)

        all_triangulated_points = np.array(all_triangulated_points)
        all_ordered_blobs = np.array(
            [
                np.vstack(np.array(all_ordered_blobs_per_frame)[:, ID])
                for ID in camera_ids
            ]
        )

        # Relation between all observations in the capture and the chosen ones
        total_observations = all_ordered_blobs.shape[1]

        # Get roughly equally spaced observations in time (and hopefully in space)
        indexes = np.arange(
            0, total_observations, total_observations // n_observations + 1
        )
        pairs_sequence = np.arange(n_observations) % len(unique_pairs)
        all_ordered_blobs = all_ordered_blobs[:, indexes]
        all_triangulated_points = np.hstack(
            [
                all_triangulated_points[p, :, i].reshape(3, -1)
                for p, i in zip(pairs_sequence, indexes)
            ]
        )

        # Condensing initial guess into initial guess parameter vector
        rvecs_tvecs = np.array(
            [
                [
                    cv2.Rodrigues(camera.extrinsic_matrix[:3, :3])[0].flatten(),
                    camera.extrinsic_matrix[:3, -1],
                ]
                for camera in self.camera_models
            ]
        )

        initial_guess = np.hstack(
            (rvecs_tvecs.flatten(), all_triangulated_points.flatten())
        )

        # Optimizing parameter vector by minimizing cost function using Levenberg–Marquardt algorithm
        result = sp.optimize.least_squares(
            fun=total_reprojection_error,
            x0=initial_guess,
            method="lm",
            args=(
                [camera.intrinsic_matrix for camera in self.camera_models],
                all_ordered_blobs,
            ),
        )

        optimized_parameters = np.array(result.x)

        # Retrieving data from optimized parameter vector
        rvecs_tvecs_adjusted = optimized_parameters[: self.n_cameras * 6].reshape(
            self.n_cameras, 2, 3
        )
        adjusted_extrinsics = [
            np.vstack(
                (np.hstack((cv2.Rodrigues(rvec)[0], tvec.reshape(3, -1))), [0, 0, 0, 1])
            )
            for rvec, tvec in rvecs_tvecs_adjusted
        ]

        # Update references
        for camera, adjusted_extrinsic in zip(self.camera_models, adjusted_extrinsics):
            camera.update_extrinsic(adjusted_extrinsic)

        # Rebuild fundamental matrices with updated references
        self.build_fundamental_matrices()

    def update_reference(self, wand_blobs, wand_distances, pair):
        # Order all wand markers
        all_triangulated_markers = []
        for sync_blobs in zip(*wand_blobs):
            triangulated_markers = self.stereo_triangulation_logic(
                pair, list(sync_blobs)
            )
            ordered_triangulated_markers = perpendicular_order(
                triangulated_markers.T, wand_distances
            )
            all_triangulated_markers.append(ordered_triangulated_markers)

        # Mean position of all wand markers
        mean_triangulated_markers = np.mean(np.array(all_triangulated_markers), axis=0)

        # Get measured O, X and Y
        O_measured = mean_triangulated_markers[0].reshape(3, -1)
        X_measured = mean_triangulated_markers[1].reshape(3, -1)
        Y_measured = mean_triangulated_markers[2].reshape(3, -1)

        # Calculate expected O, X and Y
        O_expected = np.array([[0], [0], [0]])
        X_expected = np.array([[wand_distances[0]], [0], [0]])
        Y_expected = np.array([[0], [wand_distances[1]], [0]])

        # Create point cloud
        measured = np.hstack((O_measured, X_measured, Y_measured))
        expected = np.hstack((O_expected, X_expected, Y_expected))

        # Calculate transformation and pose
        transformation = kabsch(measured, expected)

        # Update references
        for camera in self.camera_models:
            camera.update_extrinsic(np.linalg.inv(transformation @ camera.pose))

        self.build_fundamental_matrices()


def total_reprojection_error(optimize, intrinsic_matrices, all_ordered_blobs):
    # Retrieve parameters
    n_cameras = len(intrinsic_matrices)
    rvecs_tvecs = optimize[: n_cameras * 6].reshape(n_cameras, 2, 3)  # 6 DoF per Camera
    all_triangulated_points = optimize[n_cameras * 6 :].reshape(3, -1)

    # Calculate new projection matrices
    extrinsic_matrices = [
        np.hstack((cv2.Rodrigues(rvec)[0], tvec.reshape(3, -1)))
        for rvec, tvec in rvecs_tvecs
    ]
    projection_matrices = [
        I @ E for I, E in zip(intrinsic_matrices, extrinsic_matrices)
    ]

    # Compute residuals
    residuals = []
    for projection_matrix, all_detected_blobs in zip(
        projection_matrices, all_ordered_blobs
    ):
        camera_residuals = reprojection_error(
            points_to_project=all_triangulated_points,
            points_in_image=all_detected_blobs,
            projection_matrix=projection_matrix,
        )

        residuals.append(camera_residuals)

    residuals = np.concatenate(residuals)

    return residuals


def collinear_order(blobs, wand_ratio):
    # Distances between blobs
    distances = np.array(
        [
            np.linalg.norm(blobs[0] - blobs[1]),
            np.linalg.norm(blobs[1] - blobs[2]),
            np.linalg.norm(blobs[2] - blobs[0]),
        ]
    )

    min_distance = np.min(distances)

    if min_distance == 0 or np.isnan(min_distance):
        # Blobs too close may lead wrong ordering, discard data for robustness
        return np.full_like(blobs, np.nan)

    # Normalize distances
    distances /= np.min(distances)

    # Measured unique distance sums
    measured_unique_sums = np.array(
        [
            distances[0] + distances[2],
            distances[0] + distances[1],
            distances[1] + distances[2],
        ]
    )

    # Exprected unique distance sums
    expected_unique_sums = np.array(
        [
            wand_ratio[0] + wand_ratio[0] + wand_ratio[1],
            wand_ratio[0] + wand_ratio[1],
            wand_ratio[0] + wand_ratio[1] + wand_ratio[1],
        ]
    )

    # Error matrix
    difference_matrix = np.array(
        [
            [np.abs(measured - expected) for measured in measured_unique_sums]
            for expected in expected_unique_sums
        ]
    )

    # Check for ambiguities
    blob_mapping = np.argmin(difference_matrix, axis=1)
    unique_mapping = np.unique(blob_mapping)

    # Blobs to epiline correspondences are unique
    if blob_mapping.shape == unique_mapping.shape:
        return blobs[blob_mapping]

    # Blobs too close may lead wrong ordering, discard data for robustness
    return np.full_like(blobs, np.nan)


def get_other_view(
    point_reference, points_auxiliary, fundamental_matrix, collinearity_tol=0.005
):
    # Homogeneous coordinates
    point_reference_h = np.append(point_reference, 1).reshape(-1, 1)

    # Epipolar line in auxiliary view
    epiline = fundamental_matrix @ point_reference_h

    # Add homogeneous coordinates to auxiliary points
    ones = np.ones((points_auxiliary.shape[0], 1))
    points_auxiliary_h = np.hstack((points_auxiliary, ones))

    # Vectorized distance computation
    distances = np.ravel(np.abs(points_auxiliary_h @ epiline))

    # Point mapping
    matches = distances < collinearity_tol

    # Get matches within tolerance
    return matches


def max_reprojection_error(world_point, projection_matrices, image_points):
    # Homogeneous coordinates
    world_point = np.hstack((world_point, [1])).reshape(-1, 1)

    # Stack projection matrices
    projection_matrices = np.stack(projection_matrices)

    # Reproject points
    reprojected_points = (projection_matrices @ world_point).squeeze(-1)
    reprojected_points /= reprojected_points[
        :, [-1]
    ]  # Normalize homogeneous coordinates
    reprojected_points = reprojected_points[:, :-1]  # Discard the last column

    # Get maximum reprojection_error
    max_reprojection_error = np.max(
        np.linalg.norm(np.array(image_points) - reprojected_points, axis=1)
    )

    return max_reprojection_error


def triangulate_by_multivision(projection_matrices, point_in_images):
    # Generate linear system
    A = []
    for [P0, P1, P2], [u, v] in zip(projection_matrices, point_in_images):
        A.append(u * P2 - P0)
        A.append(v * P2 - P1)

    # Decompose singular values
    _, _, Vt = np.linalg.svd(A)

    world_point_h = Vt[-1]  # Get homogeneous solution
    world_point = (
        world_point_h[:3] / world_point_h[3]
    )  # Normalize homogeneous coordinates

    return world_point
