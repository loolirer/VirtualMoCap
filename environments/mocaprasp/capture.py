# Importing modules...
from picamera2 import Picamera2
from libcamera import controls
import subprocess
import pigpio
import cv2
import time
import queue
import socket
import threading
import numpy as np

from virtualmocap.vision.blob_detection import detect_blobs


# Camera Setup
try:
    picam2 = Picamera2()  # Try creating Picamera2 object

except Exception as RuntimeError:
    # If this fails, probably some other process already claimed the camera
    subprocess.run(
        ["sudo", "fuser", "-k", "/dev/video0"],  # Kills all video processes
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

resolution = (960, 720)
config = picam2.create_video_configuration(
    main={"size": resolution, "format": "YUV420"}  # Already captures in grayscale
)
picam2.configure(config)
picam2.start()  # Begin camera connection
picam2.set_controls(
    {  # Set camera controls
        "AnalogueGain": 1.0,
        "AwbEnable": False,
        "Brightness": -1.0,
        "Contrast": 32.0,
    }
)
time.sleep(1)  # Warm-up


# Socket Setup
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
    client_ip = "0.0.0.0"
    client_port = 25565
    client_address = (client_ip, client_port)
    client_socket.bind(client_address)

    print(f"[INFO] Socket created successfully")

except socket.error as err:
    print(f"[ERROR] Socket creation failed with error: {err}")

# Try searching for the server address until it is found
while True:
    try:
        server_ip = socket.gethostbyname("mocaprasp-server.local")
        server_port = 25565
        server_address = (server_ip, server_port)
        print(f"[INFO] Server found at: {server_ip}...")
        break

    except:
        print(f"[ERROR] Server not found! Retrying in 5s...")
        time.sleep(5)  # Wait for 5 seconds...
        continue


# Parallel Processes Setup
frame_queue = queue.Queue()
shot_counter = 0
lock = threading.Lock()


# GPIO controlled capture callback
def capture_callback(gpio, level, tick):
    global shot_counter
    with lock:
        shot_number = shot_counter
        shot_counter += 1

    timestamp = time.time()
    frame = picam2.capture_array()[: resolution[1], : resolution[0]]

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
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        for b in blobs:
            cv2.circle(
                frame_rgb,
                center=b[:2].astype(int),
                radius=5,
                color=(255, 0, 0),
                thickness=-1,
            )

        cv2.imshow("Camera Feed", frame_rgb)
        cv2.waitKey(1)

        message = np.append(np.ravel(blobs), [shot_number, timestamp]).astype(
            np.float32
        )
        message_bytes = message.tobytes()
        client_socket.sendto(message_bytes, server_address)

        frame_queue.task_done()


# Start the background thread
threading.Thread(target=process_and_send, daemon=True).start()


# GPIO Setup
pi = pigpio.pi()
if not pi.connected:
    raise Exception("Could not connect to pigpio daemon. Did you run 'sudo pigpiod'?")

# GPIO Pins
TRIGGER_PIN = 17  # Input: receives external trigger
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

# Set pin mode to generate PWM signal
pi.set_mode(CLOCK_PIN, pigpio.OUTPUT)


# Service loop
try:
    while True:
        # Wait for server start trigger
        print("[INFO] Waiting for server trigger...")
        message_bytes, address = client_socket.recvfrom(1024)

        # Decode message and wait for delay
        message = np.frombuffer(message_bytes, dtype=np.float32)
        delay, capture_time = message
        print(f"[INFO] Capture request received. Waiting {delay} s...")
        time.sleep(float(delay))  # Wait for delay

        print(f"[INFO] Running Capture for {capture_time} s...")
        pi.hardware_PWM(CLOCK_PIN, FREQUENCY_HZ, DUTY_CYCLE)  # Turn on capture trigger
        time.sleep(float(capture_time))  # Wait for capture time
        pi.hardware_PWM(CLOCK_PIN, 0, 0)  # Turn off capture trigger
        shot_counter = 0  # Reset shot counter for next capture

        # Clear processing queue
        with frame_queue.mutex:  # Ensure thread safety
            frame_queue.queue.clear()

        # Wait for queue to clear
        while not frame_queue.empty():
            continue

        cv2.destroyAllWindows()

except KeyboardInterrupt:
    print("\n[INFO] Exiting by external trigger...")

finally:
    pi.hardware_PWM(CLOCK_PIN, 0, 0)
    pi.stop()
    picam2.stop()
    cv2.destroyAllWindows()
