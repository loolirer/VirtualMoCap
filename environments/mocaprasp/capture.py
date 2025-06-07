from picamera2 import Picamera2
import cv2
import time
import numpy as np

# Initialize camera
picam2 = Picamera2()
# Highest FPS mode (usually 640x480 @ 90fps)
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

        print(f"{1/(finish - start)}")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    pass

cv2.destroyAllWindows()
