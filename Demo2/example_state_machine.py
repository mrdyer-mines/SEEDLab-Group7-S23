# Possible state machine for Demo 2
# State machine to separate searching, moving to, and finding the marker
# It uses a while loop to call other states from state return values

# The program begins in the start_state
def start_state():
    # Exit if we are already at the perfect position
    if(actualFPS == desiredFPS):
        return None
    # If the robot has not seen a marker, go to search state
    elif(hasSeenMarkerYet == false):
        return 1
    # If there is an error, start over from the beginning
    else:
        return 0

# Turn around in circles until a marker is found
def search_state():
    # If a marker is detected, go to move state
    if(ids):
        return 2
    # Otherwise, stay in this state and keep turning around
    else: # Keep turning
        

# Drive towards the found marker
def move_state():
    # If we stopped in front of the marker correctly, go to end_state
    if(actualFPS == desiredFPS):
        return 3
    # If the camera somehow loses sight of the marker
    elif(!ids):
        # Go back to the search state
        return 1
    # Otherwise, stay in this state and keep moving to the marker
    else: # Keep moving to it

# We are at the marker
def end_state():
    # If we need to go a few more inches
        # Code to move a few inches
        print("Made it there")
        # Exit the program
        return None
    # If we are at the marker, exit the program
    if(actualFPS == desiredFPS):
        print("Made it there")
        # Exit the program
        return None
    # If it failed to stop by the marker, go to start state
    else:
        return 0

# Variable state is initialized to 0 since 0 is the start state
state = 0

# While loop that controls the states for the length of the string
while state is not None:
    # This while loop passes returned values as the function calls
    if(state == 0):
        state = start_state()
    elif(state == 1):
        state = search_state()
    elif(state == 2):
        state = move_state(sequence)
    elif(state == 3):
        state = end_state(sequence)
    else:
        Exception("Invalid state has been entered")
