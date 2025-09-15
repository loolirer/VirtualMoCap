# Importing modules...
from picamera2 import Picamera2
from libcamera import controls
import pigpio
import cv2
import time
import queue
import atexit
import socket
import select
import threading
import numpy as np
import argparse
import os

from virtualmocap.vision.blob_detection import detect_blobs

# ROS2 IMPORT
try:
    import  rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as RosString
    from mocap_interfaces.msg import Blob, BlobArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

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

params = cv2.SimpleBlobDetector_Params()

params.minRepeatability = 3
params.minThreshold = 50
params.thresholdStep = 50
params.maxThreshold = params.minThreshold + params.thresholdStep * params.minRepeatability

params.minDistBetweenBlobs = 1
params.filterByColor = True
params.blobColor = 0

params.filterByArea = True
params.minArea = 3
params.filterByConvexity = False

marker_detector = cv2.SimpleBlobDetector_create(params)

class PiHardwareManager:
    def __init__(self):
        print("[INFO] Initializing Hardware Manager...")
        self.resolution = (960, 720)
        self.FPS = 30
        self.TIME_BUDGET = 1.0 / self.FPS

        self.frame_queue = queue.Queue()
        self.shot_counter = 0
        self.lock = threading.Lock()

        self.picam2 = Picamera2()
        
        config = self.picam2.create_video_configuration(
            main={"size": self.resolution, "format": "YUV420"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        
        self.picam2.set_controls(
            {"AeEnable": False, "AwbEnable": False, "AnalogueGain": 4.0}
        )
        print("[INFO] Camera started and configured.")
        self.pi = pigipio.pi()

        if not self.pi.connected:
            raise ConnectionError("pigpio daemon not running. Run 'sudo pigpiod'.")

        # GPIO Pins
        self.TRIGGER_PIN = 17  # Input: receives external trigger
        self.CLOCK_PIN = 18  # Output: produces trigger signal
        self.DUTY_CYCLE = 500000  # 50% duty (range: 0–1,000,000)

        # Set pin as input with pull-up/down if needed
        self.pi.set_mode(self.TRIGGER_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(self.TRIGGER_PIN, pigpio.PUD_UP)  # or PUD_DOWN

        # Glitch filter to debounce (e.g. 10000 µs = 10 ms)
        self.pi.set_glitch_filter(self.TRIGGER_PIN, 10000)

        # Set pin mode to generate PWM signal
        self.pi.set_mode(self.CLOCK_PIN, pigpio.OUTPUT)

        # Register callback on falling edge (level=0)
        self.cb = self.pi.callback(self.TRIGGER_PIN, pigpio.FALLING_EDGE, self.capture_callback)
        print("[INFO] GPIO configured for hardware trigger.")

        atexit.register(self.cleanup)

    def capture_callback(self, gpio, level, tick):
        with self.lock:
            shot_number = self.shot_counter
            self.shot_counter += 1

        timestamp = rclpy.clock.Clock().now() if (ROS2_AVAILABLE and rclpy.ok()) else time.time()
        frame = self.picam2.capture_array()[:self.resolution[1], :self.resolution[0]] 
        self.frame_queue.put((shot_number, timestamp, frame))

    def turn_on_capture(self, capture_time_s=-1):
       if capture_time_s > 0:
        print(f"[INFO] Running Capture for {capture_time_s}s...")
       else:
        print(f"[INFO] Running Capture indefinitely...")
       self.pi.hardware_PWM(self.CLOCK_PIN, self.FPS, self.DUTY_CYCLE)
    
    def turn_off_capture(self):
        print("[INFO] Turning capture OFF")
        self.pi.hardware_PWM(self.CLOCK_PIN, 0, 0)
        with self.lock:
            self.shot_counter=0
        with self.frame_queue.mutex:
            self.frame_queue.queue.clear()

    def cleanup(self):
        print("\n[INFO] Cleaning up hardware resources...")
        if hasattr(self, 'pi') and self.pi.connected:
            self.pi.hardware_PWM(self.CLOCK_PIN, 0, 0)
            self.pi.stop()
        if hasattr(self, 'picam2'):
            self.picam2.stop()
        cv2.destroyAllWindows()

class MocapClientNode(Node):
    def __init__(self, hardware_manager: PiHardwareManager):
        camera_id = os.getenv('CAM_ID', '0')
        node_name = f"Client_node_{camera_id}"
        super().__init__(node_name)
        self.get_logger().info(f"Initializing {node_name}...")

        self.hw = hardware_manager
        self.cam_id = camera_id

        self.processing_thread = threading.Thread(target=self.process_and_publish, daemon=True)
        self.processing_thread.start()

        self.blob_publisher = self.create_publisher(
            BlobArray,
            f"/cam_{self.cam_id}/blobs",
            10 # QoS depth
        )
        
        self.control_subscriber = self.create_subscription(
            RosString,
            "/mocap/capture_control",
            self.control_callback,
            10
        )

        self.get_logger().info(f"Publishing to /cam_{self.cam_id}/blobs")
        self.get_logger().info("Subscribed to /mocap/capture_control")

    def control_callback(self, msg):
        command = msg.data.upper()

        if command == "START":
            self.get_logger.info(f"Capture START request received.")
            self.hw.turn_on_capture()
        elif command == "STOP":
            self.get_logger().info(f"Capture STOP request received.")
            self.hw.turn_off_capture()
        else:
            self.get_logger().warning(f"Unknown command received: {msg.data}")

    def process_and_publish(self):
        cutoff = 100
        threshold = 127
        WINDOW_NAME = f"ROS2 Camera Feed (ID: {self.cam_id})"

        while rclpy.ok():
            try:
                shot_number, timestamp_msg, frame = self.hw.frame_queue.get(timeout=1)
            except queue.Empty:
                continue
            
            frame_proc = cv2.subtract(frame, cutoff)
            frame_proc = cv2.convertScaleAbs(frame_proc, alpha=255 / (255 - cutoff), beta=0.0)
            frame_proc = cv2.GaussianBlur(frame_proc, (3,3), 0)
            frame_proc = cv2.equalizeHist(frame_proc)

            detected_blobs = detect_blobs(frame_proc, area=True, thresh=threshold)

            msg = BlobArray()
            msg.header.stamp = timestamp_msg.to_msg()
            msg.header.frame_id = f"cam_{self.cam_id}_{shot_number}"

            frame_display = cv2.cvtColor(frame_proc, cv2.COLOR_GRAY2RGB)
            if detected_blobs.size > 0:
                for b in detected_blobs:
                    blob_msg = Blob()
                    blob_msg.x_px = float(b[0])
                    blob_msg.y_px = float(b[1])
                    blob_msg.area = float(b[2])
                    msg.blobs.append(blob_msg)
                    cv2.circle(frame_display, (int(b[0]), int(b[1])), 5, (0, 0, 255), -1)
            
            self.blob_publisher.publish(msg)
            cv2.imshow(WINDOW_NAME, frame_display)
            cv2.waitKey(1)
            self.hw.frame_queue.task_done()

            if (self.get_clock().now().nanoseconds - timestamp_msg.nanoseconds) / 1e9 > self.TIME_BUDGET:
                with self.hw.frame_queue.mutex:
                    self.hw.frame_queue.queue.clear()

def main_ros2(hw_manager):
    if not ROS2_AVAILABLE:
        print("[ERROR] rclpy or mocap_interfaces not found. Cannot run in ROS2 mode")
        return
    
    print("[INFO] Running in ROS2 mode.")
    rclpy.init()

    node = MocapClientNode(hardware_manager=hw_manager)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
def main_udp(hw_manager):
    print("[INFO] Running in UDP mode.")
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.bind(("0.0.0.0", 25565))
        print("[INFO] UDP Socket bound successfully")

    except socket.error as err:
        print(f"[ERROR] UDP Socket creation failed: {err}")
        return
    
    server_address = None
    while server_address is None:
        try:
            server_ip = socket.gethostbyname("mocaprasp-server.local")
            server_address = (server_ip, 25565)
            print(f"[INFO] Server found at: {server_ip}...")
        except socket.gaierror:
            print("[ERROR] Server not found! Retrying in 5s...")
            time.sleep(5)
    
    # Background image processing and communication
    def process_and_send_udp():
        # Lower cutoff -> + capture range / + noise
        # Higher cutoff -> - capture range / - noise
        cutoff = 100
        threshold = 127
        WINDOW_NAME = "UDP Camera Feed"

        while True:
            try:
                shot_number, timestamp, frame = hw_manager.frame_queue.get(timeout=1)

            except queue.Empty:
                continue

            # Image processing
            frame_proc = cv2.subtract(frame, cutoff)
            frame_proc = cv2.convertScaleAbs(
                frame_proc, alpha=255 / (255 - cutoff), beta=0.0
            )
            frame_proc = cv2.GaussianBlur(frame_proc, (3, 3), 0)
            frame_proc = cv2.equalizeHist(frame_proc)

            # Detect blobs
            blobs = detect_blobs(
                frame_proc, area=True, thresh=threshold, detector=marker_detector
            )
            message = np.append(np.ravel(blobs), [shot_number, timestamp]).astype(np.float32)
            client_socket.sendto(message.tobytes(), server_address)

            # Print blobs
            frame_display = cv2.cvtColor(frame_proc, cv2.COLOR_GRAY2RGB)
            if blobs.size > 0:
                for b in blobs:
                    cv2.circle(
                        frame_display,
                        center=b[:2].astype(int),
                        radius=1,
                        color=(0, 0, 255),
                        thickness=-1,
                    )

            # if shot_number == 0:
            #    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

            cv2.imshow(WINDOW_NAME, frame_display)
            cv2.waitKey(1)

            hw_manager.frame_queue.task_done()

            # Clear processing queue if time budget is exceeded
            if time.time() - timestamp > TIME_BUDGET:
                with frame_queue.mutex:  # Ensure thread safety
                    frame_queue.queue.clear()

    threading.Thread(target=process_and_send_udp, daemon=True).start()
    print("[INFO] Waiting for server trigger...")
    timeout = None

    try:
        while True:
            ready, _, _ = select.select([client_socket], [], [], timeout)

            if ready:
                try:
                    message_bytes, _ = client_socket.recvfrom(1024)

                    # Decode message and wait for delay
                    message = np.frombuffer(message_bytes, dtype=int)
                    delay, timeout = message
                    print(f"[INFO] Capture request received. Waiting {delay} s...")
                    time.sleep(float(delay))  # Wait for delay

                    hw_manager.turn_on_capture(timeout)
                    timeout = float(timeout) if timeout > 0 else None
                except (ValueError, IndexError):
                    hw_manager.turn_off_capture()
                    timeout = None
            elif timeout is not None:
                hw_manager.turn_off_capture()
                print("Capture duration finished")
                timeout = None
            ## If it reaches timeout
            #else:
            #    turn_off_capture()
            #    print("Waiting indefinetely...")
            #    timeout = None
    except KeyboardInterrupt:
        print("\n[INFO] Exiting by external trigger...")
    
def main_calibrate(hw_manager):
    print("[INFO] Running Calibration. Press Ctrl+C to exit and see results.")
    rows = []
    WINDOW_NAME = "Calibration Feed"
    try:
        while True:
            raw_frame = hw_manager.picam2.capture_array()[:hw_manager.resolution[1], :hw_manager.resolution[0]]
            blobs = detect_blob_features(raw_frame, max_area=500)
            frame_rgb = cv2.cvtColor(raw_frame, cv2.COLOR_GRAY2RGB)

            for b in blobs:
                rows.append(b)
                cv2.circle(frame_rgb, (b["cx"], b["cy"]), int(b["radius"]), (0, 255, 0), 2)
                cv2.circle(frame_rgb, (b["cx"], b["cy"]), 2, (0, 0, 255), -1)

            cv2.imshow(WINDOW_NAME, frame_rgb)
            cv2.waitKey(1)
            time.sleep(1/hw_manager.FPS)
    except KeyboardInterrupt:
        print("\n[INFO] Calibration finished. Calculating statistics...")
        if rows:
            print_calib_values(rows)
        else:
            print("[WARN] No blobs were detected during calibration")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MoCap Arena Client Application.")
    parser.add_argument('--transport', 
                        type=str, 
                        default='udp', 
                        choices=['udp','ros2'], 
                        help='Communication transport layer (for capture mode).')
    parser.add_argument('--mode', 
                        type=str,
                        default='capture',
                        choices=['capture', 'calibrate'],
                        help='Operating mode.')
    args = parser.parse_args()

    hw_manager = None
    try:
        hw_manager = PiHardwareManager()

        if args.mode == 'capture':
            if args.transport == 'ros2':
                main_ros2(hw_manager)
            else:
                main_udp(hw_manager)
        elif args.mode == 'calibrate':
            if hw_manager.cb:
                hw_manager.cb.cancel()
            main_calibrate(hw_manager)
    except (ConnectionError, KeyboardInterrupt, Exception) as e:
        print(f"\n[INFO] Shutting down due to {e}")
    finally:
        if hw_manager:
            hw_manager.cleanup()
            print("[INFO] Program terminated.")
