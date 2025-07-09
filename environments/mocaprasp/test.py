import pigpio
import time
import atexit
import sys

# --- GPIO Pin Definitions (BCM numbering) ---
TRIGGER_PIN = 17  # This pin will receive the clock signal
CLOCK_PIN = 18    # This pin will generate the clock signal

# --- Clock Parameters for Testing ---
TEST_FPS = 1      # Test with a very slow frequency (1 Hz) for easy observation
                  # You can increase this to 5, 10, or 30 later if 1Hz works.
DUTY_CYCLE_PWM = 500000  # 50% duty cycle (range is 0 to 1,000,000)

# --- Global Variables for Callback Tracking ---
last_tick = 0
trigger_count = 0
start_time = time.time()

# --- Callback Function ---
def gpio_callback(gpio, level, tick):
    """
    This function is called every time a falling edge is detected on TRIGGER_PIN.
    """
    global last_tick, trigger_count
    current_wall_time = time.time() # Wall clock time

    trigger_count += 1

    # Print general callback info
    print(f"\n[{time.strftime('%H:%M:%S', time.localtime(current_wall_time))}] "
          f"*** CALLBACK TRIGGERED! ***")
    print(f"  GPIO: {gpio}, Level: {level}, Tick: {tick} (microseconds since boot)")

    # Calculate time between triggers (using pigpio's high-resolution tick)
    if last_tick != 0:
        time_diff_us = tick - last_tick
        time_diff_s = time_diff_us / 1_000_000.0
        print(f"  Time since last trigger: {time_diff_s:.4f} seconds (Expected: {1.0/TEST_FPS:.4f}s)")
    last_tick = tick

    # Estimate actual FPS based on triggers so far
    elapsed_total_time = current_wall_time - start_time
    if elapsed_total_time > 0:
        actual_fps = trigger_count / elapsed_total_time
        print(f"  Overall Actual FPS: {actual_fps:.2f} Hz")
    print(f"  Total Triggers Received: {trigger_count}")


# --- Cleanup Function ---
def cleanup_gpio():
    """
    Ensures that GPIO is reset and pigpio connection is closed gracefully.
    """
    print("\n[INFO] Starting GPIO cleanup...")
    if 'pi' in globals() and pi.connected:
        try:
            # Stop the hardware PWM signal
            print(f"[INFO] Stopping PWM on CLOCK_PIN ({CLOCK_PIN})...")
            pi.hardware_PWM(CLOCK_PIN, 0, 0)
            # Cancel the callback
            print(f"[INFO] Cancelling callback on TRIGGER_PIN ({TRIGGER_PIN})...")
            cb.cancel()
            # Stop the pigpio library connection
            print("[INFO] Disconnecting from pigpiod daemon...")
            pi.stop()
            print("[INFO] pigpio connection stopped.")
        except Exception as e:
            print(f"[ERROR] Error during cleanup: {e}", file=sys.stderr)
    else:
        print("[INFO] No active pigpio connection to clean up.")
    print("[INFO] GPIO cleanup complete.")


# --- Register cleanup function to run on script exit ---
atexit.register(cleanup_gpio)


# --- Main Script Execution ---
if __name__ == "__main__":
    try:
        # Initialize pigpio connection
        print("[INFO] Attempting to connect to pigpiod daemon...")
        pi = pigpio.pi()  # Connect to the local Pi's pigpiod daemon

        if not pi.connected:
            raise Exception("Could not connect to pigpiod daemon. "
                            "Did you run 'sudo systemctl start pigpiod'?")
        print("[INFO] Successfully connected to pigpiod daemon.")

        # --- Configure TRIGGER_PIN (Input) ---
        print(f"[INFO] Configuring TRIGGER_PIN ({TRIGGER_PIN}) as INPUT...")
        pi.set_mode(TRIGGER_PIN, pigpio.INPUT)

        # Set pull-up resistor. This is generally good practice for inputs,
        # especially if there might be floating states, though for a direct
        # connection to a PWM output, it's less critical.
        print(f"[INFO] Setting PUD_UP on TRIGGER_PIN ({TRIGGER_PIN})...")
        pi.set_pull_up_down(TRIGGER_PIN, pigpio.PUD_UP)

        # Set glitch filter (debouncing). For a clean PWM signal,
        # a small filter or no filter might be sufficient.
        # 10000 µs = 10 ms. Try 100 µs if you suspect missed pulses at higher FPS.
        GLITCH_FILTER_US = 1000 # 1 ms filter. Adjust if needed.
        print(f"[INFO] Setting glitch filter on TRIGGER_PIN ({TRIGGER_PIN}) to {GLITCH_FILTER_US} µs...")
        pi.set_glitch_filter(TRIGGER_PIN, GLITCH_FILTER_US)

        # Register the callback function for falling edge detection
        print(f"[INFO] Registering callback on TRIGGER_PIN ({TRIGGER_PIN}) for FALLING_EDGE...")
        cb = pi.callback(TRIGGER_PIN, pigpio.FALLING_EDGE, gpio_callback)
        print("[INFO] Callback registered.")

        # --- Configure CLOCK_PIN (Output) ---
        print(f"[INFO] Configuring CLOCK_PIN ({CLOCK_PIN}) as OUTPUT...")
        pi.set_mode(CLOCK_PIN, pigpio.OUTPUT)

        # --- Start the Hardware PWM Signal ---
        print(f"\n[INFO] Starting hardware PWM on CLOCK_PIN ({CLOCK_PIN})...")
        print(f"       Frequency: {TEST_FPS} Hz")
        print(f"       Duty Cycle: {DUTY_CYCLE_PWM / 10000:.2f}%")
        pi.hardware_PWM(CLOCK_PIN, TEST_FPS, DUTY_CYCLE_PWM) # (GPIO, Frequency, DutyCycle)

        print("\n[INFO] Script is running. Look for 'CALLBACK TRIGGERED!' messages.")
        print(f"       Press Ctrl+C to stop the script and clean up GPIO.")
        print(f"       Remember to connect GPIO {CLOCK_PIN} to GPIO {TRIGGER_PIN} with a wire!")

        # Keep the script running
        while True:
            # You can add periodic status updates here if needed,
            # but the callback itself provides most of the feedback.
            time.sleep(5) # Sleep to prevent busy-waiting, callback is interrupt-driven
            print(f"[STATUS] Script active. Total triggers: {trigger_count}. (Sleeping for 5s...)")

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Exiting gracefully...")
    except Exception as e:
        print(f"\n[ERROR] An unexpected error occurred: {e}", file=sys.stderr)
        # Attempt to clean up even if an error occurs
        cleanup_gpio()

    finally:
        # cleanup_gpio will be called by atexit.register
        pass