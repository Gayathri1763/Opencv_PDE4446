import cv2
import numpy as np
import serial
import time

# ===== HSV RANGE for your object =====
LOWER_HSV = np.array([166, 80, 73])    # Your values
UPPER_HSV = np.array([179, 255, 255])
# ====================

# ===== Serial setup =====
arduino = serial.Serial('COM4', 9600, timeout=0.1)
time.sleep(2)

# ===== Open camera =====
cap = cv2.VideoCapture(1)  # Use 0 for default camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Parameters for smooth but correct tracking
Kp = 0.4       # Moderate gain
alpha = 0.3    # Some smoothing
dead_zone = 0.08  # Small dead zone
prev_move_x = 0
prev_move_y = 0

# Minimum area threshold
MIN_AREA = 200

print("Starting CORRECTED tracking...")
print("Press 'q' to quit, 'r' to reset smoothing")
print("Arrow keys: ↑↓ to adjust Kp gain, ←→ to adjust direction")

# Direction multipliers - CHANGE THESE to fix direction
direction_x = 1  # Try 1 or -1
direction_y = 1  # Try 1 or -1

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    h, w = frame.shape[:2]
    screen_center = (w // 2, h // 2)
    
    # Draw center cross
    cv2.line(frame, (screen_center[0]-10, screen_center[1]), 
             (screen_center[0]+10, screen_center[1]), (255, 255, 255), 2)
    cv2.line(frame, (screen_center[0], screen_center[1]-10), 
             (screen_center[0], screen_center[1]+10), (255, 255, 255), 2)

    # Convert to HSV and create mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    
    # Noise reduction
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    move_x = 0
    move_y = 0
    
    if len(contours) > 0:
        # Find largest contour
        largest_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_cnt)
        
        if area > MIN_AREA:
            # Get bounding box
            x, y, wb, hb = cv2.boundingRect(largest_cnt)
            obj_center = (x + wb // 2, y + hb // 2)
            
            # Draw visualization
            cv2.rectangle(frame, (x, y), (x+wb, y+hb), (0, 255, 0), 2)
            cv2.line(frame, screen_center, obj_center, (0, 255, 255), 2)
            cv2.circle(frame, obj_center, 5, (0, 0, 255), -1)
            
            # Calculate error
            error_x = obj_center[0] - screen_center[0]  # Positive = ball is RIGHT of center
            error_y = obj_center[1] - screen_center[1]  # Positive = ball is BELOW center
            
            # Normalize error
            norm_error_x = error_x / (w / 2)  # Range: -1 to +1
            norm_error_y = error_y / (h / 2)  # Range: -1 to +1
            
            # ===== CRITICAL: Direction logic =====
            # When ball is RIGHT (positive error_x), camera should pan LEFT (negative move_x)
            # When ball is LEFT (negative error_x), camera should pan RIGHT (positive move_x)
            # So we NEGATE the error!
            
            move_x = -norm_error_x * Kp * direction_x
            move_y = -norm_error_y * Kp * direction_y
            
            # Apply dead zone
            if abs(move_x) < dead_zone:
                move_x = 0
            if abs(move_y) < dead_zone:
                move_y = 0
            
            # Smooth movement
            move_x = prev_move_x * (1 - alpha) + move_x * alpha
            move_y = prev_move_y * (1 - alpha) + move_y * alpha
            prev_move_x = move_x
            prev_move_y = move_y
            
            # Clamp values
            move_x = max(-1.0, min(1.0, move_x))
            move_y = max(-1.0, min(1.0, move_y))
            
            # Display info on frame
            cv2.putText(frame, f"Error X: {error_x}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Error Y: {error_y}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Move X: {move_x:.2f}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Move Y: {move_y:.2f}", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Area: {area:.0f}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Kp: {Kp:.2f}", (w-100, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Dir X: {direction_x}", (w-100, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Dir Y: {direction_y}", (w-100, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Send command to Arduino
            formatted_x = f"{move_x:.2f}"
            formatted_y = f"{move_y:.2f}"
            command = f"{formatted_x} {formatted_y}\n"
            
            try:
                arduino.write(command.encode())
                if arduino.in_waiting > 0:
                    reply = arduino.readline().decode().strip()
                    if reply:
                        print(f"Arduino: {reply}")
                print(f"Sent: {command.strip()}")
            except Exception as e:
                print(f"Serial error: {e}")
        else:
            # Area too small - send stop
            command = "0.00 0.00\n"
            arduino.write(command.encode())
            cv2.putText(frame, "Target too small", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        # No contours - send stop
        command = "0.00 0.00\n"
        arduino.write(command.encode())
        cv2.putText(frame, "No target", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Show frames
    cv2.imshow("Tracking", frame)
    cv2.imshow("Mask", mask)
    
    # Handle keyboard input
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("Quitting...")
        break
    elif key == ord('r'):
        # Reset smoothing
        prev_move_x = 0
        prev_move_y = 0
        print("Reset smoothing")
    elif key == ord('+') or key == 82:  # Up arrow
        Kp = min(1.5, Kp + 0.1)
        print(f"Kp increased to {Kp:.2f}")
    elif key == ord('-') or key == 84:  # Down arrow
        Kp = max(0.1, Kp - 0.1)
        print(f"Kp decreased to {Kp:.2f}")
    elif key == 81:  # Left arrow - toggle X direction
        direction_x *= -1
        print(f"X direction changed to {direction_x}")
    elif key == 83:  # Right arrow - toggle Y direction
        direction_y *= -1
        print(f"Y direction changed to {direction_y}")

# Cleanup
print("Cleaning up...")
cap.release()
cv2.destroyAllWindows()

# Send stop command
try:
    arduino.write("0.00 0.00\n".encode())
    time.sleep(0.1)
    arduino.close()
    print("Arduino connection closed")
except:
    pass

print("Done!")