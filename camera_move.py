import cv2
import numpy as np
import serial
import time

# ===== HSV RANGE for your object =====
LOWER_HSV = np.array([54, 43, 0])    # adjust to your object
UPPER_HSV = np.array([179, 255, 255])
# ====================

# ===== Serial setup =====
arduino = serial.Serial('COM4', 9600, timeout=1)  # replace COM4 with your Arduino port
time.sleep(2)

# ===== Open camera =====
cap = cv2.VideoCapture(1)  # Change if needed

# Previous move values for smoothing
prev_move_x = 0
prev_move_y = 0

# Parameters
Kp = 0.3       # proportional gain (tune for speed)
alpha = 0.3    # smoothing factor
dead_zone = 0.05  # ignore tiny movements

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    screen_center = (w // 2, h // 2)
    cv2.drawMarker(frame, screen_center, (255, 255, 255), cv2.MARKER_CROSS, 10, 2)

    # Convert to HSV and create mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # Noise reduction
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Pick largest contour (assume it's the ball)
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 500:  # ignore tiny objects
            continue

        x, y, wb, hb = cv2.boundingRect(cnt)
        obj_center = (x + wb // 2, y + hb // 2)

        # Draw object
        cv2.drawMarker(frame, obj_center, (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
        cv2.rectangle(frame, (x, y), (x+wb, y+hb), (0, 255, 0), 2)
        cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 2)
        cv2.line(frame, screen_center, obj_center, (0, 255, 255), 2)

        # ----- Pixel error -----
        error_x = obj_center[0] - screen_center[0]
        error_y = obj_center[1] - screen_center[1]

        # ----- Convert to [-1, 1] and scale with Kp -----
        move_x = (error_x / (w / 2)) * Kp
        move_y = (error_y / (h / 2)) * Kp

        # Dead-zone
        if abs(move_x) < dead_zone:
            move_x = 0
        if abs(move_y) < dead_zone:
            move_y = 0

        # Smooth movement
        move_x = prev_move_x*(1-alpha) + move_x*alpha
        move_y = prev_move_y*(1-alpha) + move_y*alpha
        prev_move_x = -move_x
        prev_move_y = -move_y

        # Clamp
        move_x = max(-1, min(1, move_x))
        move_y = max(-1, min(1, move_y))

        # Round for serial stability
        move_x = round(move_x, 2)
        move_y = round(move_y, 2)

        # ===== Send to Arduino =====
        command = f"{move_x} {move_y}\n"
        arduino.write(command.encode())

        # Read Arduino reply
        reply = arduino.readline().decode().strip()
        if reply:
            print("Arduino:", reply)
        print("Sent:", command.strip())

    # Show frame and mask
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # Quit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
arduino.close()
