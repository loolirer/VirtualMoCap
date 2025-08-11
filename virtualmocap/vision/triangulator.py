import numpy as np


# Data structure for marker triangulation
class Triangulator:
    def __init__(self, multiple_view):

        # Initializing parameters
        self.multiple_view = multiple_view
        self.blobs_lists = []  # Stores every blob sent
        self.blobs_queues = []  # Stores blobs in queues for triangulation
        self.tri_idx = -1
        self.last_frame_idx = 0

        # Setup configuration
        self.reset()

    def reset(self, multiple_view=None):
        # Reset multiple view
        if multiple_view is not None:
            self.multiple_view = multiple_view

        self.blobs_lists = [[] for _ in range(self.multiple_view.n_cameras)]
        self.blobs_queues = [[] for _ in range(self.multiple_view.n_cameras)]

        self.tri_idx = -1
        self.newest_frame_idx = 0

    def save(self, id, frame_idx, blobs):
        # Log data
        self.blobs_lists[id].append((blobs, frame_idx))  # Add blobs to list

    def full_vision(self):
        frame_idxs = [list(zip(*blobs_list))[1] for blobs_list in self.blobs_lists]
        sync_frame_idxs = list(set.intersection(*map(set, frame_idxs)))
        sync_frame_idxs.sort()

        sync_blobs = []
        for blobs_list in self.blobs_lists:
            sync_blobs.append(
                [
                    frame_data[0]
                    for frame_data in blobs_list
                    if frame_data[1] in sync_frame_idxs
                ]
            )

        return sync_blobs

    def stereo_triangulation_pipeline(self, reference, frame_idx, blobs_reference):
        # Log data
        self.save(reference, frame_idx, blobs_reference)

        # Do not triangulate if triangulation is ahead from received data
        if frame_idx <= self.tri_idx:
            return None

        available_data = []
        for queue_id, blob_queue in enumerate(self.blobs_queues):
            # Ignore list on received ID
            if queue_id == reference:
                continue

            # Do not search blob queue if is empty
            if not len(blob_queue):
                continue

            try:
                queue_position = list(zip(*blob_queue))[1].index(
                    frame_idx
                )  # Get queue position of the frame index

            except:  # Did not find frame index, go to next queue
                continue

            available_data.append(
                (
                    blob_queue[queue_position][0],  # Get blobs
                    queue_id,  # Get ID of correspondent queue
                )
            )

        # If didn't find any possible triangulation
        if not available_data:
            self.blobs_queues[reference].append(
                (blobs_reference, frame_idx)
            )  # Add blobs to queue

            return None

        # If there is data available
        for triangulation_candidate in available_data:
            # Try to triangulate received data to candidate
            blobs_auxiliary, auxiliary = triangulation_candidate

            blobs_pair = [
                blobs_reference,
                blobs_auxiliary,
            ]

            triangulated_markers = self.multiple_view.stereo_triangulation_logic(
                (reference, auxiliary), blobs_pair
            )

            # Triangulation is not reliable
            if np.isnan(triangulated_markers).any():
                continue  # Try next triangulation candidate

            # If triangulation was possible, clear queue
            for queue_id, blob_queue in enumerate(self.blobs_queues):
                self.blobs_queues[queue_id] = [
                    queue_element
                    for queue_element in blob_queue
                    if queue_element[1] > frame_idx
                ]  # Only points after triangulation remains

            # Update triangulation index
            self.tri_idx = frame_idx

            # Return successfully triangulated markers
            return triangulated_markers

        return None  # No triangulation was possible with available data

    def multivision_triangulation_pipeline(
        self,
        reference,
        frame_idx,
        blobs_reference,
        max_head=2,
        max_hold=2,
        reprojection_tol=1,
        collinearity_tol=0.005,
        min_views=2,
    ):
        # Log data
        self.save(reference, frame_idx, blobs_reference)

        # Do not even add data if it is behind triangulation
        if frame_idx <= self.tri_idx:
            # Update last frame index
            if frame_idx >= self.last_frame_idx:
                self.last_frame_idx = frame_idx

            return None

        # Add received blobs to queue
        self.blobs_queues[reference].append((blobs_reference, frame_idx))

        # Build triangulation buffer
        ahead_views = 0
        to_triangulate_views = 0
        blobs_in_images = [[] for _ in range(self.multiple_view.n_cameras)]
        for queue_id, blob_queue in enumerate(self.blobs_queues):
            # Do not search if blob queue is empty
            if not len(blob_queue):
                blobs_in_images[queue_id] = np.array([])

                continue

            # Search for the last frame index in the blob queue
            try:
                queue_position = list(zip(*blob_queue))[1].index(
                    self.last_frame_idx
                )  # Get queue position of the last frame index

                to_triangulate_views += 1

            except:  # Did not find frame index, go to next queue
                blobs_in_images[queue_id] = np.array([])

                continue

            # Search for the current frame index in the blob queue
            try:
                list(zip(*blob_queue))[1].index(
                    frame_idx
                )  # Check if current frame is present

                # If so, add to ahead view counter
                ahead_views += 1

            except:  # Did not find current frame index, go to next queue
                continue

            blobs_in_images[queue_id] = blob_queue[queue_position][0]  # Get blobs

        # Still receiving messages from the triangulation target frame
        # and still waiting to triangulate with more views
        if not (ahead_views >= max_head or to_triangulate_views >= max_hold):
            # Update last frame index
            if frame_idx >= self.last_frame_idx:
                self.last_frame_idx = frame_idx

            return None

        # Triangulate markers with views
        triangulated_markers = self.multiple_view.multivision_triangulation_logic(
            blobs_in_images,
            reprojection_tol=reprojection_tol,
            collinearity_tol=collinearity_tol,
            min_views=min_views,
        )

        # No points were triangulated
        if np.isnan(triangulated_markers).any():
            # Update last frame index
            if frame_idx >= self.last_frame_idx:
                self.last_frame_idx = frame_idx

            return None

        # If triangulation was possible, clear queue
        for queue_id, blob_queue in enumerate(self.blobs_queues):
            self.blobs_queues[queue_id] = [
                queue_element
                for queue_element in blob_queue
                if queue_element[1] > self.last_frame_idx
            ]  # Only points after triangulation remains

        # Update triangulation index
        self.tri_idx = self.last_frame_idx

        # Update last frame index
        if frame_idx >= self.last_frame_idx:
            self.last_frame_idx = frame_idx

        # Return successfully triangulated markers
        return triangulated_markers
