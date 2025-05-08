# a kalman filter class for the XRP
from forwardKinematics import forwardKinematics
from XRPLib.imu import IMU
import math

class kalmanFilter:
    def __init__(self,P,Q,R,pose,dt):
        self.K = 0
        self.P = P
        self.Pk = 0
        self.Q = Q
        self.R = R
        self.x = pose
        self.xk = pose
        self.z = 0
        self.dt = dt

        self.imu = IMU.get_default_imu()
        self.fk = forwardKinematics(0.16, pose[0], pose[1], pose[2], self.dt)
        
    def measureTheta(self):
        """
        Integrates the xrp onboard imu z gyro to get angular position  
        """
        gyroz = self.imu.get_gyro_z_rate() * (math.pi/180000)
        self.z += gyroz*self.dt

    def prediction(self, vR, vL):
        """
        The prediciton step of the kalman filter

        :param vR: velocity of the right wheel
        :type vR: float
        :param vL: velocity of the left wheel
        :type vR: float  
        """
        #x_k|k-1
        x, y, theta = self.fk.kalmanPred(self.xk, vR, vL)
        self.x[0] = x
        self.x[1] = y
        self.x[2] = theta

        #P_k|k-1
        self.P = self.Pk + self.Q

    def update(self):
        """
        The update step of the kalman filter      
        """
        #getting z
        self.measureTheta()
        #K_k
        self.K = self.P / (self.P+self.R)
        #x_k|k
        self.xk[0] = self.x[0]
        self.xk[1] = self.x[1]
        self.xk[2] = self.x[2] + (self.K*(self.z - self.x[2]))
        #P_k|k
        self.Pk = (1-self.K)*self.P

    def filter(self, vR, vL):
        """
        Runs both the prediction and the update steps and returns the corresponding filtered pose

        :param vR: velocity of the right wheel
        :type vR: float
        :param vL: velocity of the left wheel
        :type vR: float        
        """
        self.prediction(vR, vL)
        self.update()
        return self.xk[0], self.xk[1], self.xk[2]