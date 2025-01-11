from sr.robot3 import *
import time, math
import util
from datetime import datetime
from pid import PID

class MyRobot:
    TARGET_MOTORS = [0,1]
    REVERSE = [1, 1]
    COUNTS_PER_REVOL = 2720
    REVOL_DIST = 0.392 #m
    ACCURACY = 100
    P, I, D = [1, 0.5, 0.5]
    ARDUINO_SERIAL = "7523031383335161A231"
    BAUD = 115200
    ROBOT = None
    MB = None
    DEBUGGER = None

    def __init__(self, window=200, revolDist = 0.392, targetMotors = [0,1], accuracy = 30, dbgEnabled = True, dbgPassThrough = False):
        self.REVOL_DIST = revolDist
        self.TARGET_MOTORS = targetMotors
        self.ACCURACY = accuracy

        self.ROBOT = Robot(raw_ports = [(self.ARDUINO_SERIAL, self.BAUD)])
        self.MB = self.ROBOT.motor_board

        self.DEBUGGER = util.MyRobotDebug(enable=dbgEnabled, passThrough=dbgPassThrough)

    def __del__(self):
        self.DEBUGGER.stop()

    def powerByCount(self, pCount, pidObject):
        return pidObject(pCount) 
    
    def RobotDrive(self, pDistance):

        self.DEBUGGER.debug(f"Started drive: pDistance: {pDistance}, targets = {self.TARGET_MOTORS}")
        #Calculate target
        targetCount = int(self.COUNTS_PER_REVOL * pDistance / self.REVOL_DIST)

        #Initialise PID objects
        m0PID = PID(self.P, self.I, self.D, setpoint = targetCount * self.REVERSE[0])
        m0PID.output_limits = (-1,1)
        m1PID = PID(self.P, self.I, self.D, setpoint = targetCount * self.REVERSE[1])
        m1PID.output_limits = (-1,1)
        
        #reset motor counts
        self.ROBOT.arduino.command("c")
        initialTime = time.time()

        reached = False

        message = ""
        while not reached:

            receivedData = self.ROBOT.arduino.command("m")
            message += f"[{time.time() - initialTime}]\n"
            if receivedData[-1] != '|':    
                message += f"Bad message received \"{receivedData}\"\n"
                continue
            
            message += f"Received Data: '{receivedData}'\n"

            m0Count, m1Count = list(map(int, receivedData[:-1].split(";")))
            if 0 in self.TARGET_MOTORS:
                self.MB.motors[0].power = (m0Power := self.powerByCount(m0Count, m0PID))
                message += f"M0: Count: {m0Count}, power: {m0Power}\n"
                reached = abs(targetCount - m0Count) < self.ACCURACY


            if  1 in self.TARGET_MOTORS:
                self.MB.motors[1].power = (m1Power := self.powerByCount(m1Count, m1PID))
                message += f"M1: Count: {m1Count}, power: {m1Power}\n"
                reached = abs(targetCount - m1Count) < self.ACCURACY

        self.DEBUGGER.debug(message)

    def RobotRotate(self, pAngle):
        arcRadius = 0.425 #m
        halfArc = arcRadius * math.pi #m

        self.RobotDrive(halfArc * (pAngle / 180) * 0.5)

    def forward(self, distance):
        self.TARGET_MOTORS = [0, 1]
        self.REVERSE = [1, 1]
        self.RobotDrive(distance)

    def reverse(self, distance):
        self.TARGET_MOTORS = [0, 1]
        self.REVERSE = [1,1]
        self.RobotDrive(-distance)

    def right(self, angle, isRadians = False):
    #    self.TARGET_MOTORS = [1]
        self.REVERSE = [-1, 1]
        if isRadians:
            self.RobotRotate(angle * (math.pi / 180))
        else:
            self.RobotRotate(angle)

    def left(self, angle, isRadians = False):
   #     self.TARGET_MOTORS = [0]
        self.REVERSE = [1, -1]
        if isRadians:
            self.RobotRotate(angle * (math.pi / 180))
        else:
            self.RobotRotate(angle)