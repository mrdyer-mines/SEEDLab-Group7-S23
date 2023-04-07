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
import smbus

# Camera calibration
camera = PiCamera()
camera.rotation = 180
rawCapture = PiRGBArray(camera)
width = 1920
height = 1088
camera.resolution = (width, height)
camera.framerate = 80
awb_mode = 'off'
camera.iso = 125
camera.brightness = 80
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Rounding for angle and distance
def round_func(x, val = 5):
    return round(x, val-int(floor(log10(abs(x)))) - 1)
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

angle = 0.0
distance = 0.0
detected = False
finalDist = 0.0
detectCount = 0
searchCount = 0

def writeMessage(angle,distance,detected):
    if(detected == False):
        try:
            bus.write_i2c_block_data(address,2,[1])
        except OSError:
            print("I2C Error.")
    else:
        angleArray=[]
        angle = float(angle)
        if angle < 0:
            posAngle = angle + 360.0
        else:
            posAngle = angle
        
        dataInAscii = [];
        dataString = 'a'+str(posAngle) + 'd'+ str(distance)
        print(dataString)
        for char in dataString:
            dataInAscii.append(ord(char))
        
        #time.sleep(0.1)
        
        try:
            bus.write_i2c_block_data(address, 0, dataInAscii)
        except OSError:
            print("I2C Error.")
        
    
    return -1
    #time.sleep(.1)


#reinitialize the encoder
#time.sleep(0.5)
bus.write_byte_data(address,1,32)
#time.sleep(0.1)

# Capture infinite frames to form a video
while(True):

    # Initialize position variables
    xvar = 0 # position of x
    yvar = 0 # position of y
    dist = 0 # distance variable

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
        
        if(searchCount == 0):
            print("Marker Not Detected")
            writeMessage(angle,finalDist,False)
            detectCount = 0
            searchCount+=1
    
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
        
        # Calculate distance using the height of the marker
        imgHeight = (abs(y1-y2)+abs(y3-y4))/2
        dist = 1/((imgHeight)/326)
        finalDist = format(((round_func(dist)) * 10), '.4f')
        
        # Calculate angle from the midpoint of the corners
        # objcenter = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
        # ratio = (objcenter - 320) / 320
        # finalAngle = format((ratio * 27), '.4f')
        
        # Calculate and continuously print angle values
        # The center of the marker is the midpoint between two corners
        objcenter = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
        # The screen length is 640 so the screen center is at 320
        ratio = (objcenter - (width / 2)) / (width /2)
        # Half the field of view for the camera is 27 degrees
        angle = ratio * 27
        
        
        
        
        
            # Output the angle and distance
        print("")
        print("Marker ID:", ids[0])
        print("Marker Angle:", str(angle).zfill(8)) # Angle in Radians
        print("Marker Distance:", str(finalDist).zfill(8)) # Distance in Meters
        writeMessage(angle,finalDist,True)
        detectCount+=1
        #searchCount +=1
# Close the program
cv2.waitKey(0)
cv2.destroyAllWindows()
