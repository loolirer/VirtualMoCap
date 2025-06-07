from picamera2 import Picamera2
import cv2
import time
import numpy as np

import sys

sys.path.append("../..")  # Go back to base directory
from modules.vision.blob_detection import detect_blobs

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
        blobs = detect_blobs(gray)
        finish = time.time()
        print(1 / (finish - start))

        if len(blobs):
            for b in blobs:
                cv2.circle(
                    frame,
                    center=b.astype(int),
                    radius=6,
                    color=(0, 0, 255),
                    thickness=-1,
                )

        cv2.imshow("Detection", frame)

except KeyboardInterrupt:
    pass

cv2.destroyAllWindows()
