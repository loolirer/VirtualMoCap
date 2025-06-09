from picamera2 import Picamera2
import pigpio
import cv2
import time
import socket
import numpy as np
import sys

sys.path.append("../..")  # Go back to base directory
from modules.vision.blob_detection import detect_blobs

# Camera setup
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (960, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
time.sleep(1)  # Warm-up


def capture_and_send(gpio, level, tick):
    frame = picam2.capture_array()
    image_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blobs = detect_blobs(image_gray, area=True)

    # Optional: Use tick or time.time()
    timestamp = time.time()
    message = np.append(np.ravel(blobs), timestamp).astype(np.float64)
    message_bytes = message.tobytes()
    client_socket.sendto(message_bytes, server_address)

    # Optional display
    # for b in blobs:
    #     cv2.circle(frame, center=b.astype(int), radius=6, color=(0, 0, 255), thickness=-1)
    # cv2.imshow("Blob Detection", frame)
    # cv2.waitKey(1)


# Start pigpio daemon and connect
pi = pigpio.pi()
if not pi.connected:
    raise Exception("Could not connect to pigpio daemon. Did you run 'sudo pigpiod'?")

# GPIO Pins
TRIGGER_PIN = 17  # Input: simulates external trigger
CLOCK_PIN = 18  # Output: produces trigger signal

# Set pin as input with pull-up/down if needed
pi.set_mode(TRIGGER_PIN, pigpio.INPUT)
pi.set_pull_up_down(TRIGGER_PIN, pigpio.PUD_UP)  # or PUD_DOWN

# Optional: glitch filter to debounce (e.g. 10000 µs = 10 ms)
pi.set_glitch_filter(TRIGGER_PIN, 10000)

# Register callback on falling edge (level=0)
cb = pi.callback(TRIGGER_PIN, pigpio.FALLING_EDGE, capture_and_send)

# Parameters
FREQUENCY_HZ = 30  # Desired frequency
DUTY_CYCLE = 500000  # 50% duty (range: 0–1,000,000)

# Set pin mode and generate PWM signal
pi.set_mode(CLOCK_PIN, pigpio.OUTPUT)
pi.hardware_PWM(CLOCK_PIN, FREQUENCY_HZ, DUTY_CYCLE)

# Socket Setup
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    print(f"[INFO] Socket created successfully")
except socket.error as err:
    print(f"[ERROR] Socket creation failed with error: {err}")

server_ip = socket.gethostbyname("loolirer.local")
server_port = 8888
server_address = (server_ip, server_port)

# Wire Output to Input
print("[INFO] Clock simulation running on GPIO27 → Trigger input on GPIO17")
print("[INFO] Press Ctrl+C to stop.")

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n[INFO] Exiting...")

finally:
    pi.hardware_PWM(CLOCK_PIN, 0, 0)
    pi.stop()
    picam2.stop()
    cv2.destroyAllWindows()
