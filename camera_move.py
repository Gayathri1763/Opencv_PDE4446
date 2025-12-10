import cv2
import numpy as np
import serial
import time

# ===== HSV RANGE for your object =====
LOWER_HSV = np.array([166, 80, 73])    # adjust to your object
UPPER_HSV = np.array([179, 255, 255])
# ====================

# ===== Serial setup =====
arduino = serial.Serial('COM4', 9600, timeout=0.1)  # Reduced timeout
time.sleep(2)

# ===== Open camera =====
cap = cv2.VideoCapture(1)  # Use 0 for default camera
# Set camera to lower resolution for faster processing
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Parameters for faster response
Kp = 0.3       # Increased for faster response
alpha = 0.2    # Reduced smoothing for faster response
dead_zone = 0.05  # Reduced dead zone
prev_move_x = 0
prev_move_y = 0

# Minimum area threshold
MIN_AREA = 200

# Skip frame counter (process every nth frame)
frame_skip = 2
frame_counter = 0

# Last time we sent command
last_send_time = time.time()
send_interval = 0.05  # Send commands every 50ms (20Hz)

print("Starting tracking...")
print("Press 'q' to quit, 'r' to reset smoothing")
print("Press 'd' to toggle direction (if still wrong)")

# Direction toggle - CHANGE THIS TO FIX DIRECTION
# If camera moves SAME direction as ball: set to -1
# If camera moves OPPOSITE direction as ball: set to 1
direction_x = -1  # Start with -1 to fix opposite movement
direction_y = 1   # Usually vertical doesn't need changing

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    frame_counter += 1
    current_time = time.time()
    
    # Skip some frames for performance
    if frame_counter % frame_skip != 0:
        continue
    
    # Resize frame for faster processing
    frame = cv2.resize(frame, (320, 240))
    
    h, w = frame.shape[:2]
    screen_center = (w // 2, h // 2)
    
    # Simple center marker (faster than drawMarker)
    cv2.line(frame, (screen_center[0]-5, screen_center[1]), 
             (screen_center[0]+5, screen_center[1]), (255, 255, 255), 1)
    cv2.line(frame, (screen_center[0], screen_center[1]-5), 
             (screen_center[0], screen_center[1]+5), (255, 255, 255), 1)

    # Convert to HSV and create mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    
    # Simple noise reduction (smaller kernel for speed)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    move_x = 0
    move_y = 0
    should_send = False
    
    if len(contours) > 0:
        # Find largest contour
        largest_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_cnt)
        
        if area > MIN_AREA:
            # Get bounding box (faster than full contour operations)
            x, y, wb, hb = cv2.boundingRect(largest_cnt)
            obj_center = (x + wb // 2, y + hb // 2)
            
            # Simple drawing (minimal for performance)
            cv2.rectangle(frame, (x, y), (x+wb, y+hb), (0, 255, 0), 1)
            cv2.line(frame, screen_center, obj_center, (0, 255, 255), 1)
            cv2.circle(frame, obj_center, 3, (0, 0, 255), -1)
            
            # Calculate error
            error_x = obj_center[0] - screen_center[0]  # Positive = ball is RIGHT of center
            error_y = obj_center[1] - screen_center[1]  # Positive = ball is BELOW center
            
            # Normalize to [-1, 1] range
            norm_error_x = error_x / (w / 2)
            norm_error_y = error_y / (h / 2)
            
            # ===== FIX: When ball is RIGHT, camera should pan LEFT =====
            # So we need NEGATIVE correlation: move_x = -norm_error_x
            # If still wrong, toggle direction_x between 1 and -1
            
            # Current logic (WRONG - same direction):
            # Ball RIGHT (+error) -> camera RIGHT (+move_x)
            
            # Desired logic (CORRECT - opposite direction):
            # Ball RIGHT (+error) -> camera LEFT (-move_x)
            
            move_x = norm_error_x * Kp * direction_x  # Added direction_x multiplier
            move_y = -norm_error_y * Kp * direction_y   # Added direction_y multiplier
            
            # Display error on screen for debugging
            cv2.putText(frame, f"Error X: {error_x:.0f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, f"Error Y: {error_y:.0f}", (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Apply dead zone
            if abs(move_x) < dead_zone:
                move_x = 0
            if abs(move_y) < dead_zone:
                move_y = 0
            
            # Light smoothing
            move_x = prev_move_x * (1 - alpha) + move_x * alpha
            move_y = prev_move_y * (1 - alpha) + move_y * alpha
            prev_move_x = move_x
            prev_move_y = move_y
            
            # Clamp values
            move_x = max(-1.0, min(1.0, move_x))
            move_y = max(-1.0, min(1.0, move_y))
            
            should_send = True
            
            # Display info on frame
            cv2.putText(frame, f"X:{move_x:.2f} Y:{move_y:.2f}", (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, f"Area:{area:.0f}", (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, f"Dir X:{direction_x}", (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    # Send commands at regular intervals (not every frame)
    if should_send and (current_time - last_send_time >= send_interval):
        # Format and send
        formatted_x = f"{move_x:.2f}"
        formatted_y = f"{move_y:.2f}"
        command = f"{formatted_x} {formatted_y}\n"
        
        try:
            arduino.write(command.encode())
            # Clear buffer before reading
            arduino.reset_input_buffer()
            
            # Try to read response without blocking too long
            if arduino.in_waiting > 0:
                reply = arduino.readline().decode().strip()
                if reply:
                    print(f"Arduino: {reply}")
            
            last_send_time = current_time
            print(f"Sent: {command.strip()} | Error: ({error_x:.0f}, {error_y:.0f})")
        except Exception as e:
            print(f"Serial error: {e}")
    
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
    elif key == ord('+'):
        Kp = min(1.0, Kp + 0.05)
        print(f"Kp increased to {Kp:.2f}")
    elif key == ord('-'):
        Kp = max(0.05, Kp - 0.05)
        print(f"Kp decreased to {Kp:.2f}")
    elif key == ord('d'):
        # Toggle direction
        direction_x *= -1
        print(f"Direction X toggled to: {direction_x}")

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