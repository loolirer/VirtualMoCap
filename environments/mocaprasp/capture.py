from picamera2 import Picamera2
import cv2
import time
import socket
import numpy as np

import sys
sys.path.append('../..') # Go back to base directory

from modules.vision.blob_detection import detect_blobs

# Try to create client socket
try: 
    client_socket = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP
    print(f'[INFO] Socket created successfully')
    
except socket.error as err: 
    print(f'[ERROR] Socket creation failed with error: {err}')

server_ip = socket.gethostbyname("loolirer.local") # Target IP
server_port = 8888 # UDP Port
server_address = (
    server_ip, 
    server_port
)

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
        # Capture image and get blobs
        start = time.time()
        frame = picam2.capture_array()  # Direct NumPy array
        image_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blobs = detect_blobs(image_gray)
        finish = time.time()
        print(1 / (finish - start))

        # Send blob data
        blobs = detect_blobs(image_gray, area=True)
        message = np.append(np.ravel(blobs), time.time()).astype(np.float32)
        message_bytes = message.tobytes()
        client_socket.sendto(message_bytes, server_address)

        #if len(blobs):
        #    for b in blobs:
        #        cv2.circle(
        #            frame,
        #            center=b.astype(int),
        #            radius=6,
        #            color=(0, 0, 255),
        #            thickness=-1,
        #        )
        #cv2.imshow("Blob Detection", frame)


except KeyboardInterrupt:
    pass

cv2.destroyAllWindows()