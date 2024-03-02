# Importing modules...
import numpy as np
import cv2

def detect_blobs(image, threshold):
    # Convert to grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Convert the grayscale image to binary image
    _, image_thresh = cv2.threshold(image_gray, threshold, 255, cv2.THRESH_BINARY)

    # Find blob contours
    contours, _ = cv2.findContours(image_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    blob_centroids = None

    if contours: # Check if empty
        for contour in contours:
            
            moments = cv2.moments(contour) # Calculate moments of each contour
            
            # Calculate x,y coordinate of center
            if moments['m00'] != 0:
                u = int(moments['m10'] / moments['m00'])
                v = int(moments['m01'] / moments['m00'])

                centroid = np.array([[u], 
                                     [v]])
                
            else:
                continue

            if blob_centroids is None:
                blob_centroids = centroid
            else:
                blob_centroids = np.hstack((blob_centroids, centroid))

    return blob_centroids