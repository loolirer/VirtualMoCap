from picamera2 import Picamera2
import cv2
import time

# Initialize camera
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    controls={"FrameDurationLimits": (11111, 11111)}  # ~90 fps
)
picam2.start()

# Allow camera to warm up
time.sleep(1)

while True:
    start = time.time()

    frame = picam2.capture_array()  # Direct NumPy array
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    finish = time.time()

    print(1 / (finish - start))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
