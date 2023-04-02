# Find the angle and distance to an aruco marker
# Use the picamera with opencv and numpy
import cv2
import cv2.aruco as aruco
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import math
from math import floor
from math import log10

# Camera calibration
camera = PiCamera()
camera.rotation = 180
rawCapture = PiRGBArray(camera)
width = 1920
height = 1088
camera.resolution = (width, height)
camera.framerate = 45
awb_mode = 'off'
camera.iso = 125
camera.brightness = 80
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Rounding for angle and distance
def round_func(x, val = 5):
    return round(x, val-int(floor(log10(abs(x)))) - 1)

# Capture infinite frames to form a video
while(True):

    # Initialize position variables
    xvar = 0 # position of x
    yvar = 0 # position of y
    midvar = 0 # center position
    anglevar = 0 # angle position
    radvar = 0 # radians position
    dist1 = 0 # first distance
    dist2 = 0 # second distance

    # Start the camera feed
    camera.capture(rawCapture, format = "bgr")
    image = rawCapture.array
    rawCapture.truncate(0)

    # Resize the image and find its properties
    resize = cv2.resize(image, None, fx=1, fy=1, interpolation = cv2.INTER_LINEAR)
    grayImage = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImage, aruco_dict, parameters = parameters)

    # If no marker is detected
    if not len(corners) > 0:
        print("Marker Not Detected")
    
    # If a marker is detected
    # Calculate the position variables
    else:
        for markers in corners:
            for marker in markers:
                for corner in marker:
                    xvar += corner[0]
                    yvar += corner[1]
                xvar /= 4
                yvar /= 4
        y1 = corners[0][0][0][0]
        y2 = corners[0][0][2][0]
        y3 = corners[0][0][1][0]
        y4 = corners[0][0][3][0]
        
        # Calculate the height, center and angle
        imgHeight = (abs(y1-y2)+abs(y3-y4))/2
        midvar = xvar - (width/2)
        anglevar = (((midvar/(width/2))*30))
        radvar = anglevar*(math.pi/180)

        # Round the final values
        dist1 = 1/((imgHeight)/326)
        finalAngle = round_func(radvar)
        finalDist = round_func(dist1)
        
        # Output the angle and distance
        print("")
        print("Marker ID:", ids[0])
        print("Marker Angle:", finalAngle) # Angle in Radians
        print("Marker Distance:", finalDist) # Distance in Meters

# Close the program
cv2.waitKey(0)
cv2.destroyAllWindows()