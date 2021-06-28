import numpy as np
from utils.abstractRobotHal import RobotHAL

class Solo12(RobotHAL):
    ''' Define the hardware interface to solo12'''

    def __init__(self, interfaceName="", dt=0.001):
        RobotHAL.__init__(self, interfaceName, dt)

    def InitRobotSpecificParameters(self):
        ''' Definition of the Solo12 paramters '''
        self.nb_motors = 12
        self.motorToUrdf = [0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10]
        self.gearRatio = np.array(self.nb_motors * [9., ])  # gearbox ratio
        self.motorKt = np.array(self.nb_motors * [0.025, ])  # Nm/A
        self.motorSign = np.array([-1, +1, -1, -1, +1, +1,
                                   -1, +1, -1, -1, +1, +1])
        self.maximumCurrent = 12.0  # A
        # To get this offsets, run the calibration with self.encoderOffsets at 0,
        # then manualy move the robot in zero config, and paste the position here (note the negative sign!)
        self.encoderOffsets = - np.array([-1.7551466226577759, 1.8692125082015991, -0.33785927295684814,
                                          -2.8217074871063232, 2.4837048053741455, -2.5392541885375977,
                                          -1.7476146221160889, -2.1298675537109375, 1.9545761346817017,
                                           2.2128634452819824, -3.4803080558776855, -1.0698696374893188])

        # self.encoderOffsets *= 0.
        # 180 degree roll
        # self.rotateImuVectors = lambda x: [-x[0], -x[1], x[2]]
        # self.rotateImuOrientation = lambda q: [q[3], -q[2], -q[1], q[0]]

        # IMU oriented like the base (no rotation)
        self.rotateImuVectors = lambda x: [x[0], x[1], x[2]]
        self.rotateImuOrientation = lambda q: [q[3], -q[2], q[1], -q[0]]


