from forwardKinematics import forwardKinematics
from kalman import kalmanFilter
from trials import move, moveCircle, pointTurn
from XRPLib.board import Board
import time
import math
from machine import Timer

#initializing forward kinematics class with wheel
sampleTime = 0.02 #s
d = 0.16
fK = forwardKinematics(d, 0, 0, 0, sampleTime)
kFilt = kalmanFilter(0.1, 0.01, 0.01, [0,0,0], sampleTime)


######################
##### trial 1 ########
######################

#straight at 200 mm/s for 3 seconds
move(3, 200, 200, fK, kFilt)
#point turn at 500 mm/s for 5 seconds
move(5, 500, -500, fK, kFilt)
#straight at 300 mm/s for 3 seconds
move(3, 300, 300, fK, kFilt)

