# PDE 4446 Course Work 2: OpenCV Git Repo and Demo

## Introduction
The project here is to detect a red ball given and align it to the center of the screen. Basically this is a coursework which uses object detection using OpenCV. The course work is done step by step. 

## Step 1: Find Out the HSV Values of the Ball
I created a code in Python that allows us to create a slider box which adjusts the HSV colour space of the camera screen by ourselves. By sliding each of the Lower and higher HSV we will isolate the colour RED where the screen only detects red objects. The HSV can vary depending on the lighting condition of the room.

## Step 2: Use HSV Values to Detect the Object
The object detection process is continued by adding this HSV manually into a code which opens up the real camera screen and output screen of the masked object (the red objects).  
Here all the red objects in front of the screen will be shown in white and the rest will be in black.

## Step 3: Create a Contour of the Object and find the centers
Now we add contour lines for the object that is detected. As its easy to detect the center of shapes like rectangle, square etc we create a rectangle around the contour lines.  
There will be issue when there are other red objects in the screen, so we edit the code as it only detects the object which is larger in size. So while placing the nearer to camera it will be the largest one and will be detected.  
Next we found out the center of this rectangle and also the center of camera screen and placing two diff colour dots in each centers.

## Step 4: Finding the Distance Between Two Centers
Now we find the distance between the center of object and center of camera. We also draw a line between them.  
Our next goal is to find the difference and adjust it as they align with each other.

## Step 5: The Final: Camera Control using OpenCV and PySerial
## Pan–Tilt Camera Object Tracking 

## System Pipeline

1. Camera captures live video frames.
2. Python (OpenCV):
   - Converts the frame from BGR to HSV color space
   - Detects the colored object using HSV thresholding
   - Calculates the pixel error between object center and frame center
   - Normalizes the error to the range [-1, 1]
   - Applies proportional gain, dead-zone, and smoothing
3. Python sends pan–tilt commands to Arduino via Serial (9600 baud).
4. Arduino:
   - Receives "pan tilt" values as a string
   - Maps values to servo angles
   - Sends back "OK" after updating servos

---

## References

- https://www.geeksforgeeks.org/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-with-cv-inrange-opencv/?utm_source=chatgpt.com
- ChatGPT, Deepseek
- Google Web Browser

## Conclusion

This project successfully demonstrates the integration of vision-based sensing with closed-loop motion control for real-time object tracking.

Outcomes:
- Vision-based sensing using OpenCV and HSV color segmentation
- Closed-loop control that continuously aligns the camera with the object
- Stable serial communication between Python and Arduino for actuator control

The video presentation :-  https://youtu.be/1VE17JQMzdc?si=7kAB63myiUmQANPD


