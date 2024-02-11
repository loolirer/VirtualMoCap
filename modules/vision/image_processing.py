# Importing modules...
import numpy as np
import cv2

def detect_blobs(image, threshold):
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Convert the grayscale image to binary image
    _, thresh_image = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY)

    # Find blob contours
    contours, _ = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    blob_centroids = None
    blob_areas = None

    if contours: # Check if empty
        for contour in contours:
            
            moments = cv2.moments(contour) # Calculate moments of each contour
            
            # Calculate x,y coordinate of center
            if moments['m00'] != 0:
                x = int(moments['m10'] / moments['m00'])
                y = int(moments['m01'] / moments['m00'])

                centroid = np.array([[x], 
                                     [y]])
                
                area = np.array([moments['m00']])
            else:
                continue

            if blob_centroids is None:
                blob_centroids = centroid
                blob_areas = area
            else:
                blob_centroids = np.hstack((blob_centroids, centroid))
                blob_areas = np.append(blob_areas, area)

    return blob_centroids, blob_areas