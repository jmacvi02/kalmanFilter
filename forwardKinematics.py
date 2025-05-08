# A forward kinematics class for differantial drive robots
import math


class forwardKinematics:
    def __init__(self, wheelBase, x, y, q, dt):
        self.d = wheelBase
        self.dt = dt
        self.x = x
        self.y = y
        self.q = q
        self.r = 0
        self.v = 0
        self.w = 0
        self.straight = True

    def velCalcs(self, vR, vL):
        """
        Calculate the distance from the ICC, and store it in the class

        :param vR: velocity of the right wheel
        :type vR: float
        :param vL: velocity of the left wheel
        :type vR: float        
        """
       
        if abs(vR - vL) < 0.0001: #accounting for small difference in velocity while going straight
            self.straight = True
            self.v = vR 
        else:
            self.straight = False
            self.w = (vR-vL) / self.d
            self.r = (self.d*(vR+vL)) / (2*(vR-vL))

    
    def updatePos(self, vR, vL):
        """
        Calculate the distance from the ICC, and store it in the class

        :param vR: velocity of the right wheel
        :type vR: float
        :param vL: velocity of the left wheel
        :type vR: float   
        :return x: updated x position 
        :return y: updated y position     
        :return q: updated angular position     
        """
        self.velCalcs(vR,vL)

        if self.straight:
            self.x += self.v*math.cos(self.q)*self.dt
            self.y += self.v*math.sin(self.q)*self.dt
            #self.q = self.q
        else:
            self.x += -self.r*math.sin(self.q) + self.r*math.sin(self.q+self.w*self.dt)
            self.y += +self.r*math.cos(self.q) - self.r*math.cos(self.q+self.w*self.dt)
            self.q += self.w*self.dt
        
        return self.x, self.y, self.q
    
    def kalmanPred(self, pose, vR, vL):
        self.x = pose[0]
        self.y = pose[1]
        self.q = pose[2]

        return self.updatePos(vR, vL)
