from XRPLib.encoder import Encoder
from XRPLib.motor import Motor
from XRPLib.board import Board
import math 
import time

#global variables uses by the class
encoderLPinA = 4 #encoder pins
encoderLPinB = 5
encoderRPinA = 12
encoderRPinB = 13

motorLPinA = 6 #motor pins
motorLPinB = 7
motorRPinA = 14
motorRPinB = 15

counts_per_wheel_revolution = (30/14) * (28/16) * (36/9) * (26/8) * 12 # 585
wheel_diameter = 60 #mm
circumference_wheel = math.pi*wheel_diameter #188.49mm

# Name:         PID class
# Description:  This class takes a users desired PID parameters and can control the XRP robot's
#               Wheel velocities. The class depends the 2 of the XRP libraries given classes
#               (the motor and encoder classes).
class PIDLinVelCont:
    def __init__(self, Kp, Ki, Kd, side):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.side = side
        if (side.lower() == 'l'):
            self.encoder = Encoder(0,encoderLPinA,encoderLPinB)
            self.motor = Motor(motorLPinA, motorLPinB, True)
        elif (side.lower() == 'r'):
            self.encoder = Encoder(1,encoderRPinA,encoderRPinB)
            self.motor = Motor(motorRPinA, motorRPinB)
        self.error_sum = 0
        self.error_delta = 0
        self.prev_error = 0
        self.prev_count = 0

    # Name:     calcEffort
    # Purpose:  Determine the new effort to send to the motor
    # Inputs:   The current error and the time since the last update
    # Returns:  The effort to send to the motor
    def calcEffort(self, error, wait_time):
        self.error_delta = (error - self.prev_error)
        self.error_sum += error 
        self.prev_error = error

        effort = self.Kp*error+ self.Kd*self.error_delta + self.Ki*self.error_sum

        return effort

    # Name:     calcVelocity
    # Purpose:  Deterimine the linear velocity of the wheel
    # Inputs:   The time passed since the previous calculation
    # Returns:  The linear velocity of the wheel
    def calcVelocity(self, wait_time):  
        #getting the position change since the last check
        currentCount = self.encoder.get_position_counts()
        #calculation count change
        deltaCount = currentCount - self.prev_count
        #storing previous count 
        self.prev_count = currentCount
        #calculating ang velocity
        angVelocity = deltaCount*2*math.pi / (counts_per_wheel_revolution * wait_time)
        #calc linear velocity
        if (self.side.lower() == 'l'):
            velocity = (-1)* angVelocity * (wheel_diameter/2) #sign flipped to align encoder with motor + direction
        elif (self.side.lower() == 'r'):
            velocity = angVelocity * (wheel_diameter/2)

        return velocity
    
    # Name:     updateEffort
    # Purpose:  Update the effort of the wheel depending on the specified control law
    # Inputs:   Current linear velocity of the wheel, the target velocity, time since last update
    # Returns:  The linear velocity of the wheel
    def updateEffort(self, velTarget, velocity, wait_time):
        #finding the difference between current velocity and target velocity
        error =  velTarget - velocity
        #using the speed controller class to determine the effort to send the motor
        effort = self.calcEffort(error, wait_time)
        #sending the effort calculated by speed controller class to each motor
        self.motor.set_effort(effort)

        return