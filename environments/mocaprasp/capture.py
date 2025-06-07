from picamzero import picam2, start_preview, stop_preview
import time

# Start camera preview
start_preview()

print("Camera preview running. Press Ctrl+C to stop.")

try:
    while True:
        # Capture a frame as a numpy array (if needed)
        frame = picam2.capture_array()
        # Do something with the frame (optional)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping preview...")

finally:
    # Stop the preview when exiting
    stop_preview()