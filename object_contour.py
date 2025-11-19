import cv2
import numpy as np

# ===== ENTER YOUR HSV RANGE =====
LOWER_HSV = np.array([175, 59, 75])
UPPER_HSV = np.array([179, 239, 255])
# =================================

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask 
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # Reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find ALL contours 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw largest contour and bounding boxes
    if len(contours) > 0:
        # Pick largest contour by area
        cnt = max(contours, key=cv2.contourArea)
        # Draw the contour (blue)
        cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 2)

        # Get bounding box
        x, y, w, h = cv2.boundingRect(cnt)

        # Draw rectangle (green)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)


    # Display output
    cv2.imshow("All Contours", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
