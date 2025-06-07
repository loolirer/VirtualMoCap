import cv2

# Open default camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Check if it was opened correctly
if not cap.isOpened():
    print("[ERROR] Could not open camera")
    exit()

while True:
    # Read frame
    ret, frame_rgb = cap.read()

    if not ret:
        print("[ERROR] Could not retrieve frame")
        break

    # Frame display
    cv2.imshow("Camera feed", frame_rgb)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Close camera and destroy all windows
cap.release()
cv2.destroyAllWindows()
