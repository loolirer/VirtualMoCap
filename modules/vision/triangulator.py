import numpy as np

# Data structure for marker triangulation
class Triangulator:
    def __init__(self, multiple_view):

        # Initializing parameters
        self.multiple_view = multiple_view
        self.blobs_lists = [] # Stores every blob sent
        self.blobs_queues = [] # Stores blobs in queues for triangulation
        self.tri_idx = -1

        # Setup configuration
        self.reset()

    def reset(self, multiple_view=None):
        # Reset multiple view
        if multiple_view is not None:
            self.multiple_view = multiple_view

        self.blobs_lists = [[] for _ in range(self.multiple_view.n_cameras)]
        self.blobs_queues = [[] for _ in range(self.multiple_view.n_cameras)]

        self.tri_idx = -1

    def save(self, id, frame_idx, blobs):
        # Log data
        self.blobs_lists[id].append(
            (blobs, frame_idx)
        )  # Add blobs to list

    def full_vision(self):
        frame_idxs = [list(zip(*blobs_list))[1] for blobs_list in self.blobs_lists]
        sync_frame_idxs = list(set.intersection(*map(set, frame_idxs)))
        sync_frame_idxs.sort()

        sync_blobs = []
        for blobs_list in self.blobs_lists:
            sync_blobs.append([frame_data[0] for frame_data in blobs_list if frame_data[1] in sync_frame_idxs])

        return sync_blobs

    def triangulate(self, reference, frame_idx, blobs_reference):
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

            # Do not search blob queue is empty
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

            triangulated_markers = self.multiple_view.triangulate_by_pair(
                (reference, auxiliary), blobs_pair
            )

            # Triangulation is not reliable
            if np.isnan(triangulated_markers).any():
                continue # Try next triangulation candidate

            # If triangulation was possible, clear queue
            for queue_id, blob_queue in enumerate(self.blobs_queues):
                self.blobs_queues[queue_id] = [
                    queue_element
                    for queue_element in blob_queue
                    if queue_element[1] > frame_idx
                ]  # Only points after triangulation remains

            # Update triangulation index
            self.tri_index = frame_idx

            # Return successfully triangulated markers
            return triangulated_markers
        
        return None # No triangulation was possible with available data