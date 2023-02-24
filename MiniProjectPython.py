# 
import smbus 
import time
# Include the board packges for lcd screen
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# Include the cv2 and aruco packages
import cv2 as cv
import cv2.aruco as aruco

#Set address
#ser = serial.Serial('/dev/ttyACM0', 115200)


# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeMessage(quadrant):
    print("sending")
    bus.write_byte_data(address,0,quadrant)
    time.sleep(.1)
    #time.sleep(1)
    
#def ReadfromArduino():
#    while (ser.in_waiting > 0):
#        try:
#            line = ser.readline().decode('utf-8').rstrip()
 #           print("serial output : ", line)
            
#       except:
 #           print("Communication Error")
#
# This code detects Aruco markers in real-time
# It prints which quadrant they are in (1-4)

# Start the video capture feed
cam = cv.VideoCapture(0)
if not cam.isOpened():
    print("Camera not opened")
    exit()
    
# Set up the aruco dictionary and parameters
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
p = aruco.DetectorParameters_create()
current_id = "[[]]"
trackMarker = 0
quadrant = 0

# Get the length and width of the frame
print("Frame length:",cam.get(3))
print("Frame width:", cam.get(4))

# Half the height of the frame
horizontal_cutoff = cam.get(4) / 2
# Half the length of the frame
vertical_cutoff = cam.get(3) / 2






if quadrant == 0:
    #send specific offset to reset encoders 
    bus.write_byte_data(address,1,quadrant)
#ser.write(quadrant.encode())    
# Capture infinite frames to form a video
while True:
        
    #current = bus.read_i2c_block_data(address,2,1)
    #print(current)
    # Check for errors
    ret, frame = cam.read()
    if not ret:
        print("Frame not received")
        break
    
    # Stop the program when a is pressed
    if cv.waitKey(1) == ord('a'):
        break
    
    # Show the continuous video feed in grayscale
    output = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow('frame', output)
    
    # Look for aruco markers
    (corners, ids, rejected) = aruco.detectMarkers(output, arucoDict, parameters = p)
    # If one or more markers are found
    if len(corners) > 0:
        
        # Display the ID of the marker in the frame
        # If the current except: marker is not the same as the previous one
        if current_id != str(ids[0]) or trackMarker == 1:
            # For each marker that the camera is looking at
            for x in range(0, len(ids)):
                # Print its ID, but never twice in a row
                print("")
                print("Marker ID:", ids[x])
                # Will always hold the most recently printed ID
                current_id = str(ids[x])
                trackMarker = 0
        
        # x-coordinate calculations for comparison with vertical_cutoff
        left_side = min(corners[0][0][0][0], corners[0][0][1][0], corners[0][0][2][0], corners[0][0][3][0])
        right_side = max(corners[0][0][0][0], corners[0][0][1][0], corners[0][0][2][0], corners[0][0][3][0])
        # y-coordinate calculations for comparison with horizontal_cutoff
        top_side = max(corners[0][0][0][1], corners[0][0][1][1], corners[0][0][2][1], corners[0][0][3][1])
        bottom_side = min(corners[0][0][0][1], corners[0][0][1][1], corners[0][0][2][1], corners[0][0][3][1])
        
        
        # Clears the lcd
        lcd.clear()
        if bottom_side > horizontal_cutoff:
            if left_side > vertical_cutoff:
                print("Quadrant 4")
                quadrant = 4
                bus.write_byte_data(address,0,quadrant)
                #ser.write(quadrant.encode())
                lcd.message = "3pi/2"
            if right_side < vertical_cutoff:
                print("Quadrant 3")
                quadrant = 3
                bus.write_byte_data(address,0,quadrant)
                #ser.write(quadrant.encode())
                lcd.message = "pi"
        if top_side < horizontal_cutoff:
            if right_side < vertical_cutoff:
                print("Quadrant 2")
                quadrant = 2
                bus.write_byte_data(address,0,quadrant)
                #ser.write(quadrant.encode())
                lcd.message = "pi/2"
            if left_side > vertical_cutoff:
                print("Quadrant 1")
                quadrant = 1
                bus.write_byte_data(address,0,quadrant)
                #ser.write(quadrant.encode())
                lcd.message = "0"
               
    # Print not detected when the marker is no longer seen
    if not len(corners) > 0:
        if trackMarker == 0:
            print("")
            print("No Marker Detected")
            trackMarker = 1
    
    #ReadfromArduino()
    

# Close the frame
cam.release()
cv.destroyAllWindows()


