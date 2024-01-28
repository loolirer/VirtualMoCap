# Importing modules...
import numpy as np
import cv2

def detect_blobs(image, threshold):
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Convert the grayscale image to binary image
    _, thresh_image = cv2.threshold(gray_image, threshold, 255, 0)

    # Find blob contours
    contours, _ = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    blob_centroids = None

    if contours: # Check if empty
        for contour in contours:
            M = cv2.moments(contour) # Calculate moments of each contour
            
            # Calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                centroid = np.array([[cX], 
                                     [cY]])
            else:
                break

            if blob_centroids is None:
                blob_centroids = centroid
            else:
                blob_centroids = np.hstack((blob_centroids, centroid))

    return blob_centroids