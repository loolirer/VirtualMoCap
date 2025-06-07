from picamera2 import Picamera2
import time
import cv2

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")

# Start the camera
picam2.start()
time.sleep(2)  # Sllow sensor to settle

# Capture an image
image = picam2.capture_array()

# Show the image using OpenCV
cv2.imshow("Captured Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()