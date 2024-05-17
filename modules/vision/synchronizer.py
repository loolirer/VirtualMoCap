import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.interpolate import CubicSpline

def proximity_order(previous_blobs, current_blobs):    
    distance_matrix = np.array([[np.linalg.norm(previous_blob - current_blob) 
                                for current_blob in current_blobs] 
                                for previous_blob in previous_blobs])

    # Using the hungarian (Munkres) assignment algorithm to find unique correspondences between blobs
    _, new_indices = linear_sum_assignment(distance_matrix)

    return current_blobs[new_indices]

# Data structure for data interpolation
class Synchronizer:
    def __init__(self, 
                 blob_count=1, # Number of expected blobs for interpolation
                 window=3,  # The minimum ammount of data points for interpolating 
                 step=0.05, # Time step for interpolation in seconds
                 capture_time=10 # Capture time in seconds
                 ):
        
        # Initializing interpolation parameters
        self.blob_count = blob_count
        self.interpolation_window = window
        self.step = step
        self.capture_time = capture_time
        self.interpolation_start = 0

        # Raw data - how it comes from the clients
        self.async_PTS = []
        self.async_blobs = []

        # Interpolated data - how it should be triangulated
        self.sync_PTS = np.arange(0.0, self.capture_time, self.step)
        self.sync_blobs = np.full((self.sync_PTS.size, blob_count, 2), -1.0) # Non-interpolated blobs are negative
       
    def add_data(self, blobs, PTS):
        # Do not add data if PTS is out of recording range
        if PTS > self.capture_time:
            return False # Data refused

        # Check if it's not empty
        if self.async_PTS:
            # Do not add data if incoming PTS is lesser or equal than the last PTS added
            # - Assure strictly ascending order of time
            # - Avoid sequenced repeated messaging
            if PTS <= self.async_PTS[-1]:
                return False # Data refused

        # Ordering blobs of this message by their proximity to the others on the previous message 
        if self.async_blobs: # If there are blobs
            # Try to order blobs
            ordered_blobs = proximity_order(self.async_blobs[-1], blobs)

            self.async_blobs.append(ordered_blobs)
            self.async_PTS.append(PTS)

        else: # If there aren't any blobs, just add them
            self.async_blobs.append(blobs)
            self.async_PTS.append(PTS)

        # Enough points to interpolate in the same blob ordering?
        if len(self.async_PTS) >= self.interpolation_window:
            # Get the window last blob coordinate data
            async_PTS = np.array(self.async_PTS[-self.interpolation_window:])
            async_blobs = np.array(self.async_blobs[-self.interpolation_window:])
            
            start = self.interpolation_start    # Start index of interpolated PTS  
            end = int(async_PTS[-1] // self.step) # Final index of interpolated PTS  

            for blob in range(self.blob_count):
                # Extracting blob coordinate history in the interpolation window
                # The slicing works like: async_blob = async_blobs[PTS, blob, axis]
                async_blob = async_blobs[:, blob, :]

                # Generating a cubic spline that represents blob trajectory 
                blob_trajectory = CubicSpline(async_PTS, async_blob)
                
                # Get blob tracjectory in the interpolated timestamps that are not yet interpolated
                interpolated_blob = blob_trajectory(self.sync_PTS[start:end+1]) # End of slice is exclusive!

                # Add interpolated blobs to data structure
                self.sync_blobs[start:end+1, blob, :] = interpolated_blob

            # Updating interpolation start to the next PTS 
            self.interpolation_start = end + 1

        return True # Data accepted