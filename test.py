import cv2
import matplotlib.pyplot as plt
import numpy as np

# Create a window for the video
cv2.namedWindow("Grayscale Video", cv2.WINDOW_NORMAL)

# Setup webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open webcam")
    exit()

# Setup matplotlib figure for histogram
plt.ion()  # interactive mode on
fig, ax = plt.subplots()
(hist_orig,) = ax.plot(np.zeros(256), color="gray")
(hist_proc,) = ax.plot(np.zeros(256), color="black")
ax.set_xlim([0, 256])
ax.set_ylim([0, 10000])
ax.set_title("Live Histogram")
ax.set_xlabel("Pixel Intensity")
ax.set_ylabel("Frequency")

# Real-time loop
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert to grayscale
        orig = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        proc = cv2.blur(orig, (11, 11))

        # Display video
        cv2.imshow("Grayscale Video", orig)

        # Compute histogram
        hist = cv2.calcHist([orig], [0], None, [256], [0, 256]).flatten()
        hist_lpf = np.convolve(hist, np.ones(9)/9, mode='same')
        hist_orig.set_ydata(hist)
        hist_proc.set_ydata(hist_lpf)
        fig.canvas.draw()
        fig.canvas.flush_events()

        # Break with ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    plt.ioff()
    plt.show()
