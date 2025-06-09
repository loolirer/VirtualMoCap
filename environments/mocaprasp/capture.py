from picamera2 import Picamera2
import cv2
import time
import socket
import numpy as np
import RPi.GPIO as GPIO
import threading
import sys

sys.path.append("../..")  # Go back to base directory
from modules.vision.blob_detection import detect_blobs

# GPIO Pins
TRIGGER_PIN = 17  # Input: simulates external trigger
CLOCK_PIN = 27  # Output: simulates external clock

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(CLOCK_PIN, GPIO.OUT)
GPIO.output(CLOCK_PIN, GPIO.LOW)

# Socket Setup
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    print(f"[INFO] Socket created successfully")
except socket.error as err:
    print(f"[ERROR] Socket creation failed with error: {err}")

server_ip = socket.gethostbyname("loolirer.local")
server_port = 8888
server_address = (server_ip, server_port)

# Camera setup
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (960, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
time.sleep(1)  # Warm-up


# --- Triggered Capture Handler ---
def capture_and_send(channel):
    start = time.time()
    frame = picam2.capture_array()
    image_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blobs = detect_blobs(image_gray)
    finish = time.time()
    print(f"FPS: {1 / (finish - start):.2f}")

    blobs = detect_blobs(image_gray, area=True)
    message = np.append(np.ravel(blobs), time.time()).astype(np.float32)
    message_bytes = message.tobytes()
    client_socket.sendto(message_bytes, server_address)

    # Optional display
    # for b in blobs:
    #     cv2.circle(frame, center=b.astype(int), radius=6, color=(0, 0, 255), thickness=-1)
    # cv2.imshow("Blob Detection", frame)
    # cv2.waitKey(1)


# Attach interrupt to GPIO trigger pin
GPIO.add_event_detect(
    TRIGGER_PIN, GPIO.FALLING, callback=capture_and_send, bouncetime=10
)


# Clock Simulator Thread
def gpio_clock_simulator(freq_hz=30.0):
    period = 1.0 / freq_hz
    half_period = period / 2
    while True:
        GPIO.output(CLOCK_PIN, GPIO.HIGH)
        time.sleep(half_period)
        GPIO.output(CLOCK_PIN, GPIO.LOW)
        time.sleep(half_period)


# Start GPIO clock simulation in a thread
clock_thread = threading.Thread(target=gpio_clock_simulator, args=(30.0,), daemon=True)
clock_thread.start()

# Wire Output to Input
print("[INFO] Clock simulation running on GPIO27 → Trigger input on GPIO17")
print("[INFO] Press Ctrl+C to stop.")

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n[INFO] Exiting...")

finally:
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
