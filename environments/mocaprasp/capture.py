from picamera2 import Picamera2
import pigpio
import sys
import cv2
import time
import queue
import socket
import threading
import numpy as np

sys.path.append("../..")  # Go back to base directory
from modules.vision.blob_detection import detect_blobs

# Camera setup
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (960, 720), "format": "Y8"} # Already captures in grayscale
)
picam2.configure(config)
picam2.start()
time.sleep(1)  # Warm-up


# Socket setup
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    print(f"[INFO] Socket created successfully")
except socket.error as err:
    print(f"[ERROR] Socket creation failed with error: {err}")

server_ip = socket.gethostbyname("loolirer.local")
server_port = 8888
server_address = (server_ip, server_port)


# Parallel processes setup
frame_queue = queue.Queue()
shot_counter = 0
lock = threading.Lock()


# ===== GPIO Callback: Capture Only =====
def capture_callback(gpio, level, tick):
    global shot_counter
    with lock:
        shot_number = shot_counter
        shot_counter += 1

    timestamp = time.time()
    frame = picam2.capture_array()

    # Push to processing queue
    frame_queue.put((shot_number, timestamp, frame))


# Background image processing and communication
def process_and_send():
    while True:
        try:
            shot_number, timestamp, frame = frame_queue.get(timeout=1)
        except queue.Empty:
            continue

        blobs = detect_blobs(frame, area=True)

        message = np.append(np.ravel(blobs), [shot_number, timestamp]).astype(
            np.float64
        )
        message_bytes = message.tobytes()
        client_socket.sendto(message_bytes, server_address)

        frame_queue.task_done()


# Start the background thread
threading.Thread(target=process_and_send, daemon=True).start()


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
cb = pi.callback(TRIGGER_PIN, pigpio.FALLING_EDGE, capture_callback)

# Clock parameters
FREQUENCY_HZ = 30  # Desired frequency
DUTY_CYCLE = 500000  # 50% duty (range: 0–1,000,000)

# Set pin mode and generate PWM signal
pi.set_mode(CLOCK_PIN, pigpio.OUTPUT)
pi.hardware_PWM(CLOCK_PIN, FREQUENCY_HZ, DUTY_CYCLE)

# Showcase info
print("[INFO] Clock simulation running on GPIO18 → Trigger input on GPIO17")
print("[INFO] Press Ctrl+C to stop.")

# Stall loop
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
