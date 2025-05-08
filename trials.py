# A set of functions to help make the moves for trial in main.
# Utilizes the PID class tuned from lab 1 to stabily move at specified velocities. 
# prints both the forward kinematic model predictions and the kalman filter model predictions 
from PID import PIDLinVelCont
from forwardKinematics import forwardKinematics
from kalman import kalmanFilter
from XRPLib.board import Board
import time
import math

sampleTime = 0.02#s, choose your sampling time
Kp_left = 0.002 #PID speed control
Ki_left = 0.0002
Kd_left = 0.001 
Kp_right = Kp_left
Ki_right = Ki_left
Kd_right = Kd_left
d = 0.16

leftWheel = PIDLinVelCont(Kp_left, Ki_left, Kd_left,'l')
rightWheel = PIDLinVelCont(Kp_right, Ki_right, Kd_right, 'r')
leftWheel.motor.set_effort(0)
rightWheel.motor.set_effort(0)
board = Board.get_default_board()

#note an only turns counter clockwise and for positive relRadians values
def pointTurn(relRadians, velTarget, fK):
    """
    Execute a counterclockwise point turn of a specifed relative angle at a specified speed for the XRP. 
    Also updates a forwad kinematics model for the XRP accordingly.

    :param relRadians: the angle to turn from the current angle (only makes move for angles >0)
    :type relRadians: float
    :param velTarget: magnitude of velocity for each wheel (in mm/s)
    :type velTarget: float      
    :param fk: instance of the forwardKinematics class to update
    :type fk: class     
    """
    print("point turn")
    
    initQ = fK.q
    rightVelTarget = velTarget
    leftVelTarget  = -rightVelTarget # equal but opposite target velocities for point turn
    initTime = time.ticks_ms()
    lastTime = initTime
    currentTime = time.ticks_ms()

    #printcount = 0 # used to how many pose values to print
    while fK.q < relRadians + initQ: #
        if board.is_button_pressed():  # Check if the button is pressed to stop
            print("Stopped!")
            leftWheel.motor.set_effort(0)
            rightWheel.motor.set_effort(0)
            board.wait_for_button()  # Wait for button release to avoid immediate restart
            break
        
        currentTime = time.ticks_ms()
        waitTime = (currentTime - lastTime)/1000 # convert from ms to s
        if waitTime >= sampleTime:
            #finding current linear velocity of each wheel
            leftVel = leftWheel.calcVelocity(waitTime) 
            rightVel = rightWheel.calcVelocity(waitTime)
            # updating the 
            x, y, q = fK.updatePos(rightVel/1000, leftVel/1000)
            # uncomment this to print the current x and y positions while moving
            #if printcount == 2:
                #print(f"{x}, {y}")
                #printcount = 0
            #printcount += 1
            #updating the effort according to the control law
            leftWheel.updateEffort(leftVelTarget, leftVel, waitTime)
            rightWheel.updateEffort(rightVelTarget, rightVel, waitTime)
            lastTime = currentTime
            
    leftWheel.motor.set_effort(0)
    rightWheel.motor.set_effort(0)        

def move(duration, rightVelTarget, leftVelTarget, fK, kFilt):
    """
    set the XRP by setting target velocities for each wheel and specifying a time to move. 
    Also updates a forwad kinematics model for the XRP accordingly.

    :param duration: the time the XRP will move at target velocity
    :type duration: float
    :param rightVelTarget: target velocity for right wheel (in mm/s)
    :type rightVelTarget: float      
    :param leftVelTarget: target velocity for left wheel (in mm/s)
    :type leftVelTarget: float    
    :param fk: instance of the forwardKinematics class to update
    :type fk: class     
    """
    initTime = time.ticks_ms()
    lastTime = initTime
    currentTime = time.ticks_ms()
    printcount = 0 # used to how many pose values to print
    # main loop
    while (currentTime - initTime)/1000 < duration:
        if board.is_button_pressed():  # Check if the button is pressed to stop
            print("Stopped!")
            leftWheel.motor.set_effort(0)
            rightWheel.motor.set_effort(0)
            board.wait_for_button()  # Wait for button release to avoid immediate restart
            break
        
        currentTime = time.ticks_ms()
        waitTime = (currentTime - lastTime)/1000 # convert from ms to s
        if waitTime >= sampleTime:
            #finding current linear velocity of each wheel
            leftVel = leftWheel.calcVelocity(waitTime) 
            rightVel = rightWheel.calcVelocity(waitTime)
            xF, yF, qF = fK.updatePos(rightVel/1000, leftVel/1000)
            xK, yK, qK = kFilt.filter(rightVel/1000, leftVel/1000)

            if printcount == 10: #uncomment to print x and y position
                printcount = 0
                print(f"fK,{xF}, {yF}")
                print(f"kalF, {xK}, {yK}")
            printcount += 1
            #updating the effort according to the control law
            leftWheel.updateEffort(leftVelTarget, leftVel, waitTime)
            rightWheel.updateEffort(rightVelTarget, rightVel, waitTime)
            lastTime = currentTime

    leftWheel.motor.set_effort(0)
    rightWheel.motor.set_effort(0)

def moveCircle(lapDuration, radius, fK, kFilt):
    """
    Move the xrp in 1 circle of a specified radius. Uses the move function to execute the movement

    :param lapDuration: the time the XRP will take to complete the cirlce
    :type lapDuration: float
    :param radius: the radius of the circle (in m)
    :type radius: float      
    :param fk: instance of the forwardKinematics class to update
    :type fK: forwardKinematics    
    """
    print("moving circle")
    # calulating the velocities to send to each motor. 
    w = 2*math.pi / lapDuration
    rightVelTarget = w*(radius + (d/2))
    leftVelTarget = w*(radius - (d/2))

    move(lapDuration, rightVelTarget*1000, leftVelTarget*1000, fK, kFilt)

    return