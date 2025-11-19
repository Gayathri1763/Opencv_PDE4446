import cv2
import numpy as np

# ======= ENTER YOUR HSV VALUES HERE =======
# Example values (you MUST change these to your values)
LOWER_HSV = np.array([175, 59, 75])   # lower range
UPPER_HSV = np.array([179, 239, 255])   # upper range
# =========================================

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # Show video and mask
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Detected Mask (White = Object)", mask)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
