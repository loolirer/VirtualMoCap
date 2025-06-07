from picamera2 import Picamera2
import cv2
import time

# Initialize camera
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(main={"format": "RGB888", "size": (960, 720)})
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
