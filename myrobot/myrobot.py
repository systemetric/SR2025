from sr.robot3 import *
import time, math
import util
from datetime import datetime
from pid import PID

class MyRobot:
    WINDOW = 200
    REVOL_DIST = 0.392
    TARGET_MOTORS = [0,1]

    def __init__(self, window=200, revolDist = 0.392, targetMotors = [0,1]):
        self.WINDOW = window
        self.REVOL_DIST = revolDist
        self.TARGET_MOTORS = targetMotors

    def InternalDrive(self):
        pass

    def forward(self, distance):
        pass

    def reverse(self, distance):
        pass

    def right(self, angle):
        pass

    def left(self, angle):
        pass