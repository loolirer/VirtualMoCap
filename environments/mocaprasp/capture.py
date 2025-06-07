from picamzero import Camera
from time import sleep

cam = Camera()
cam.start_preview()
# Keep the preview window open for 5 seconds
sleep(5)