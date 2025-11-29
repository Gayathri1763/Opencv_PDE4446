# PDE 4446 Course Work 2: OpenCV Git Repo and Demo

## Introduction
The project here is to detect a red ball given and align it to the center of the screen. Basically this is a coursework which uses object detection using OpenCV. The course work is done step by step. 

## Step 1: Find Out the HSV Values of the Ball
I created a code in Python that allows us to create a slider box which adjusts the HSV values of the camera screen by ourselves. By sliding each of the Lower and higher HSV we will isolate the colour RED where the screen only detects red objects. The HSV can vary depending on the lighting condition of the room.

## Step 2: Use HSV Values to Detect the Object
The object detection process is continued by adding this HSV values into a code which opens up the real camera screen and output screen of the masked object (the red objects).  
Here all the red objects in front of the screen will be shown in white and the rest will be in black.

## Step 3: Create a Contour of the Object
Now we add contour lines for the object that is detected. As its easy to detect the center of shapes like rectangle, square etc we create a rectangle around the contour lines.  
There will be issue when there are other red objects in the screen, so we edit the code as it only detects the object which is larger in size. So while placing the nearer to camera it will be the largest one and will be detected.  
Next we found out the center of this rectangle and also the center of camera screen and placing two diff colour dots in each centers.
