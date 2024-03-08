# Importing modules...
import numpy as np
import cv2

params = cv2.SimpleBlobDetector_Params()

# Check if blob is stable in the three filters
params.minRepeatability    = 3

# Three threshold filters
params.minThreshold        = 50
params.thresholdStep       = 50
params.maxThreshold        = params.minThreshold + params.thresholdStep * params.minRepeatability

# Minimum distance between blobs is 1 pixel
params.minDistBetweenBlobs = 1

# Filter only dark blobs
params.filterByColor       = True
params.blobColor           = 0

# Filter only blobs with over 2 pixels
params.filterByArea        = True
params.minArea             = 2

# Do not filter by convexity to allow distorted blobs to be detected
params.filterByConvexity   = False

# Instanciate marker detector object
marker_detector = cv2.SimpleBlobDetector_create(params)

def detect_blobs(image, detector=marker_detector):
    # Apply threshold to image
    thresh = 127
    _, image_thresh = cv2.threshold(image, thresh, 255, cv2.THRESH_BINARY_INV)

    # Optimization: finding a smaller sub-image that contains all blobs
    zero_pixels = np.array(np.where(image_thresh == 0))

    # Sub-image new corners
    margin = 5
    u_min, u_max = min(zero_pixels[1]) - margin, max(zero_pixels[1]) + margin
    v_min, v_max = min(zero_pixels[0]) - margin, max(zero_pixels[0]) + margin

    # Detect keypoints in sub-image
    keypoints = detector.detect(image_thresh[v_min:v_max, u_min:u_max])

    # Make detected blobs matrix
    detected_blobs = None
    for k in keypoints:
        blob_centroid = np.array([[k.pt[0] + u_min], 
                                  [k.pt[1] + v_min]])

        if detected_blobs is None:
            detected_blobs = blob_centroid
        else:
            detected_blobs = np.hstack((detected_blobs, blob_centroid))

    detected_blobs = detected_blobs.astype(int) # Cast as interger

    return detected_blobs