# Importing modules...
from picamera2 import Picamera2
import pigpio
import cv2
import time
import queue
import atexit
import socket
import threading
import numpy as np

from virtualmocap.vision.blob_detection import detect_blobs


def detect_blob_features(
    gray_img,
    min_area=1,
    max_area=1000,
    min_circularity=0.1,
    min_convexity=0.1,
    min_inertia=0.1,
    thresh=127,
):

    _, binary = cv2.threshold(gray_img, thresh, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (min_area <= area <= max_area):
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        # Circularity
        circularity = 4 * np.pi * area / (perimeter**2)
        if circularity < min_circularity:
            continue

        # Convexity
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        convexity = area / hull_area
        if convexity < min_convexity:
            continue

        # Inertia Ratio
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        mu20, mu02 = M["mu20"], M["mu02"]
        inertia_ratio = min(mu20, mu02) / max(mu20, mu02) if max(mu20, mu02) > 0 else 0
        if inertia_ratio < min_inertia:
            continue

        # Enclosing circle
        _, radius = cv2.minEnclosingCircle(cnt)

        blobs.append(
            {
                "cx": cx,
                "cy": cy,
                "radius": radius,
                "area": area,
                "circularity": circularity,
                "convexity": convexity,
                "inertia": inertia_ratio,
            }
        )

    return blobs


def print_calib_values(rows):
    # Set print options to display floats with 2 decimal places
    np.set_printoptions(precision=2, suppress=True)

    features = ["radius", "area", "circularity", "convexity", "inertia"]
    dataframe = np.array([[r[f] for f in features] for r in rows]).T

    for feature, data in zip(features, dataframe):
        Q1 = np.quantile(data, 0.25)
        Q2 = np.median(data)
        Q3 = np.quantile(data, 0.75)
        IQR = Q3 - Q1

        upper_bound = Q3 + (1.5 * IQR)
        lower_bound = Q1 - (1.5 * IQR)

        upper_whisker = np.max(np.compress(data <= upper_bound, data))
        lower_whisker = np.min(np.compress(data >= lower_bound, data))

        outliers = data[(data < lower_whisker) | (data > upper_whisker)]

        total_measures = len(data)
        valid_measures = total_measures - len(outliers)

        print(f"{feature.upper()}: ({valid_measures}/{total_measures})")
        print(f"\tUpper Whisker: {upper_whisker:.2f}")
        print(f"\tUpper Box: {Q3:.2f}")
        print(f"\tMedian: {Q2:.2f}")
        print(f"\tLower Box: {Q1:.2f}")
        print(f"\tLower Whisker: {lower_whisker:.2f}")


# Blob detector parameters
params = cv2.SimpleBlobDetector_Params()

# Check if blob is stable in the three filters
params.minRepeatability = 3

# Three threshold filters
params.minThreshold = 50
params.thresholdStep = 50
params.maxThreshold = (
    params.minThreshold + params.thresholdStep * params.minRepeatability
)

# Minimum distance between blobs in pixels
params.minDistBetweenBlobs = 1

# Filter only dark blobs
params.filterByColor = True
params.blobColor = 0

# Filter only blobs with over 4 pixels
params.filterByArea = False
params.minArea = 1.0
params.maxArea = 40.0

# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.59
params.maxCircularity = 1.00

# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.88
params.minConvexity = 1.00

# Filter by Inertia
params.filterByInertia = False

# Instanciate marker detector object
marker_detector = cv2.SimpleBlobDetector_create(params)


# Camera setup
picam2 = Picamera2()  # Create Picamera2 object

FPS = 30  # In hertz
TIME_BUDGET = 1.0 / FPS  # In seconds
EXPOSURE_TIME = 5000  # In microseconds

resolution = (960, 720)
config = picam2.create_video_configuration(
    main={
        "size": resolution,
        "format": "RGB888",
    }  # Already captures in grayscale
)
picam2.configure(config)
picam2.start()  # Begin camera connection
picam2.set_controls(
    {  # Set camera controls
        "AeEnable": False,
        "AwbEnable": False,
        # "Brightness": 1.0,
        # "AnalogueGain": 10.0,
        # "Contrast": 32.0,
        # "ExposureTime": EXPOSURE_TIME,
    }
)

time.sleep(1)  # Warm-up


# Socket Setup
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
    print(f"[INFO] Socket created successfully")

    client_ip = "0.0.0.0"
    client_port = 25565
    client_address = (client_ip, client_port)
    client_socket.bind(client_address)

    print(f"[INFO] Socket bound successfully")

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
    frame = picam2.capture_array()

    # Push to processing queue
    frame_queue.put((shot_number, timestamp, frame))


# Background image processing and communication
def process_and_send():
    while True:
        try:
            shot_number, timestamp, frame_rgb = frame_queue.get(timeout=1)
        except queue.Empty:
            continue

        frame_gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        frame_contrast = cv2.convertScaleAbs(frame_gray, alpha=4.0, beta=-200.0)
        blobs = detect_blobs(
            frame_contrast, area=True, thresh=127, detector=marker_detector
        )
        frame_display = cv2.cvtColor(frame_contrast, cv2.COLOR_GRAY2RGB)

        for b in blobs:
            cv2.circle(
                frame_display,
                center=b[:2].astype(int),
                radius=5,
                color=(0, 0, 255),
                thickness=-1,
            )

        cv2.imshow("Camera Feed", frame_display)
        cv2.waitKey(1)

        message = np.append(np.ravel(blobs), [shot_number, timestamp]).astype(
            np.float32
        )
        message_bytes = message.tobytes()
        client_socket.sendto(message_bytes, server_address)

        frame_queue.task_done()

        # Clear processing queue if time budget is exceeded
        if time.time() - timestamp > TIME_BUDGET:
            with frame_queue.mutex:  # Ensure thread safety
                frame_queue.queue.clear()


rows = []


def blob_calib():
    while True:
        try:
            shot_number, timestamp, frame = frame_queue.get(timeout=1)
        except queue.Empty:
            continue

        blobs = detect_blob_features(frame, max_area=500)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        for b in blobs:
            rows.append(b)
            cx = b["cx"]
            cy = b["cy"]
            r = int(b["radius"])
            cv2.circle(frame_rgb, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(frame_rgb, (cx, cy), 2, (0, 0, 255), -1)

        cv2.imshow("Round Blob Detection", frame_rgb)
        cv2.waitKey(1)

        frame_queue.task_done()


# Start the background thread
# threading.Thread(target=process_and_send, daemon=True).start()
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

# Glitch filter to debounce (e.g. 10000 µs = 10 ms)
pi.set_glitch_filter(TRIGGER_PIN, 10000)

# Register callback on falling edge (level=0)
cb = pi.callback(TRIGGER_PIN, pigpio.FALLING_EDGE, capture_callback)

# Clock parameters
DUTY_CYCLE = 500000  # 50% duty (range: 0–1,000,000)

# Set pin mode to generate PWM signal
pi.set_mode(CLOCK_PIN, pigpio.OUTPUT)


# Cleanup setup
def cleanup():
    pi.hardware_PWM(CLOCK_PIN, 0, 0)  # Turns off clock
    pi.stop()  # Stop pigipio daemon
    picam2.stop()  # Kill camera connection
    cv2.destroyAllWindows()  # Destroy OpenCV windows


atexit.register(cleanup)


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

        rows = []  # Reset rows

        print(f"[INFO] Running Capture for {capture_time} s...")
        pi.hardware_PWM(CLOCK_PIN, FPS, DUTY_CYCLE)  # Turn on capture trigger
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

        if rows:
            print_calib_values(rows)

except KeyboardInterrupt:
    print("\n[INFO] Exiting by external trigger...")
