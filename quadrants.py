# This code detects Aruco markers in real-time
# It prints which quadrant they are in (1-4)

# Include the cv2 and aruco packages
import cv2 as cv
import cv2.aruco as aruco

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

# Get the length and width of the frame
print("Frame length:", cam.get(3))
print("Frame width:", cam.get(4))

# Half the height of the frame
horizontal_cutoff = cam.get(4) / 2
# Half the length of the frame
vertical_cutoff = cam.get(3) / 2

# Capture infinite frames to form a video
while True:
    
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
        # If the current marker is not the same as the previous one
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
        
        if bottom_side > horizontal_cutoff:
            if left_side > vertical_cutoff:
                print("Quadrant 4")
            if right_side < vertical_cutoff:
                print("Quadrant 3")
        if top_side < horizontal_cutoff:
            if right_side < vertical_cutoff:
                print("Quadrant 2")
            if left_side > vertical_cutoff:
                print("Quadrant 1")
                
    # Print not detected when the marker is no longer seen
    if not len(corners) > 0:
        if trackMarker == 0:
            print("")
            print("No Marker Detected")
            trackMarker = 1

# Close the frame
cam.release()
cv.destroyAllWindows()