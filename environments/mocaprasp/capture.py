from picamera2 import Picamera2
import cv2
import time
import numpy as np


params = cv2.SimpleBlobDetector_Params()

# Check if blob is stable in the three filters
params.minRepeatability    = 3

# Three threshold filters
params.minThreshold        = 50
params.thresholdStep       = 50
params.maxThreshold        = params.minThreshold + params.thresholdStep * params.minRepeatability

# Minimum distance between blobs in pixels
params.minDistBetweenBlobs = 1

# Filter only dark blobs
params.filterByColor       = True
params.blobColor           = 0

# Filter only blobs with over 2 pixels
params.filterByArea        = True
params.minArea             = 3

# Do not filter by convexity to allow distorted blobs to be detected
params.filterByConvexity   = False

# Instanciate marker detector object
marker_detector = cv2.SimpleBlobDetector_create(params)

def detect_blobs(image, area=False, detector=marker_detector):
    # Apply threshold to image
    thresh = 127
    _, image_thresh = cv2.threshold(image, thresh, 255, cv2.THRESH_BINARY_INV)

    # Optimization: finding a smaller sub-image that contains all blobs
    zero_pixels = np.array(np.where(image_thresh == 0))

    # Image blacked out after thresh
    if not zero_pixels.size:
        return np.array([])

    # Sub-image new corners
    margin = 5 # Arbitrary margin for sub-image  
    u_min, u_max = min(zero_pixels[1]) - margin, max(zero_pixels[1]) + margin
    v_min, v_max = min(zero_pixels[0]) - margin, max(zero_pixels[0]) + margin

    # Clip values to not go beyond the original image limits
    u_min, u_max = np.clip([u_min, u_max], 0, image_thresh.shape[1] - 1)
    v_min, v_max = np.clip([v_min, v_max], 0, image_thresh.shape[0] - 1)

    # Sub-image reference 
    sub_image_origin = np.array([u_min, v_min])

    # Detect keypoints in sub-image
    keypoints = detector.detect(image_thresh[v_min:v_max+1, u_min:u_max+1]) # End of slice is exclusive!

    # No valid blob found!
    if not keypoints:
        return np.array([])

    # If valid blobs were found, make detected blobs matrix in the original image reference
    detected_blobs = np.array([k.pt for k in keypoints]) + sub_image_origin

    # Append blob areas to the matrix if needed
    if area:
        blob_areas = np.array([k.size for k in keypoints])
        detected_blobs = np.hstack((detected_blobs, blob_areas.reshape(-1, 1)))

    return detected_blobs

# Initialize camera
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (960, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

# Allow camera to warm up
time.sleep(1)

try:
    while True:
        start = time.time()
        frame = picam2.capture_array()  # Direct NumPy array
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        finish = time.time()

        print(1/(finish-start)) 

        blobs = detect_blobs(gray)

        for b in blobs:
            cv2.circle(
                frame,
                center=detect_blobs(gray),
                radius=6,
                color=(0, 0, 255),
                thickness=-1
            )

        cv2.imshow("Detection", frame)
    
        if cv2.waitKey(1) == ord('q'):
            break

except KeyboardInterrupt:
    pass

cv2.destroyAllWindows()
