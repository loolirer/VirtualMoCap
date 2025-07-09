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

FPS = 30

# Parallel Processes Setup
frame_queue = queue.Queue()
shot_counter = 0
lock = threading.Lock()


# GPIO controlled capture callback
def capture_callback(gpio, level, tick):
    print("A")

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

pi.hardware_PWM(CLOCK_PIN, FPS, DUTY_CYCLE)  # Turn on capture trigger
time.sleep(float(10))  # Wait for capture time
pi.hardware_PWM(CLOCK_PIN, 0, 0)  # Turn off capture trigger

# Clear processing queue
with frame_queue.mutex:  # Ensure thread safety
    frame_queue.queue.clear()

# Wait for queue to clear
while not frame_queue.empty():
    continue
