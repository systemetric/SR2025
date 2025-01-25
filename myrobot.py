from sr.robot3 import *
import time, math
import util
from datetime import datetime
from pid import PID

class MyRobot:
    __TARGET_MOTORS = [0,1]
    __REVERSE = [1, 1]
    __COUNTS_PER_REVOL = 2720
    __REVOL_DIST = 0.392 #m
    __ACCURACY = 30
    __P, __I, __D = [1, 0.05, 0.3]
    #__P, __I, __D = [1, 1, 0.1]
    __ARDUINO_SERIAL = "7523031383335161A231"
    __BAUD = 115200
    ROBOT = None
    __MB = None
    __DEBUGGER = None

    def __init__(self, window=200, revolDist = 0.392, targetMotors = [0,1], accuracy = 30, dbgEnabled = True, dbgPassThrough = False):
        self.__REVOL_DIST = revolDist
        self.__TARGET_MOTORS = targetMotors
        self.__ACCURACY = accuracy

        self.ROBOT = Robot(raw_ports = [(self.__ARDUINO_SERIAL, self.__BAUD)])
        self.__MB = self.ROBOT.motor_board

        self.__DEBUGGER = util.MyRobotDebug(enable=dbgEnabled, passThrough=dbgPassThrough)

    def __del__(self):
        self.__DEBUGGER.stop()

    def stop(self):
        self.__MB.motors[0].power = 0
        self.__MB.motors[1].power = 0

    def __setReached(self, motors = [0,1]):
        if 0 in motors:
            self.__MB.motors[0].power = 0
        if 1 in motors:
            self.__MB.motors[1].power = 0
        pass

    def __powerByCount(self, pCount, pidObject):
        return pidObject(pCount)

    def __pidOverMotors(self, m0Count, m1Count):
        m0Count, m1Count = list(map(abs,[m0Count, m1Count]))
        pDelta = m0Count - m1Count
        pSum = m0Count + m1Count
        if abs(pDelta) < 25:
            return 1, 1
        factor = 1 if pSum == 0 else 1 - (abs(pDelta) ** 2) / pSum
        if pDelta > 0:
            return factor, 1
        elif pDelta < 0:
            return 1, factor
        else:
            return 1, 1
        
    def __clamp(self, pmin, x, pmax):
        return max(min(x, pmax), pmin)

    def __RobotDrive(self, pDistance):
        self.__DEBUGGER.debug(f"Started drive: pDistance: {pDistance}, targets = {self.__TARGET_MOTORS}")
        #Calculate target
        targetCount = int(self.__COUNTS_PER_REVOL * pDistance / self.__REVOL_DIST)

        self.__DEBUGGER.debug(f"Going to {targetCount}...\n")

        #Initialise PID objects
        m0PID = PID(self.__P, self.__I, self.__D, setpoint = targetCount * self.__REVERSE[0])
        m0PID.output_limits = (-1,1)
        m1PID = PID(self.__P, self.__I, self.__D, setpoint = targetCount * self.__REVERSE[1])
        m1PID.output_limits = (-1,1)

        mXPID = PID(1, 0, 0, setpoint = 0)
        mXPID.output_limits = (-1,1)
        
        #reset motor counts
        self.ROBOT.arduino.command("c")
        initialTime = time.time()

        m0reached = False
        m1reached = False

        m0LastCount = 0
        m1LastCount = 1

        lastTime = initialTime

        message = ""
        while not m0reached or not m1reached:

            receivedData = self.ROBOT.arduino.command("m")
            message += f"[{time.time() - initialTime}]\n"
            if receivedData[-1] != '|':    
                message += f"Bad message received \"{receivedData}\"\n"
                continue
            
            message += f"Received Data: '{receivedData}'\n"

            m0Count, m1Count = list(map(int, receivedData[:-1].split(";")))

            factorM0, factorM1 = self.__pidOverMotors(m0Count, m1Count)
            message += f"M0 Factor: {factorM0}\nM1Factor: {factorM1}\n"

       #     self.__DEBUGGER.debug(f"{time.time()},{m0Count},{m1Count}")

            if 0 in self.__TARGET_MOTORS and not m0reached:
                self.__MB.motors[0].power = (m0Power := self.__clamp(-1, factorM0 * self.__powerByCount(m0Count, m0PID), 1))
                message += f"M0: Count: {m0Count}, power: {m0Power}\n"
                m0reached = abs(abs(targetCount) - abs(m0Count)) < self.__ACCURACY
                self.__DEBUGGER.debug(abs(abs(targetCount) - abs(m0Count)))
                if m0reached:
                    message += "Stopped M0\n"
                    self.__setReached(motors=[0])

            if  1 in self.__TARGET_MOTORS and not m1reached:
                self.__MB.motors[1].power = (m1Power := self.__clamp(-1, factorM1 * self.__powerByCount(m1Count, m1PID), 1))
                message += f"M1: Count: {m1Count}, power: {m1Power}\n"
                m1reached = abs(abs(targetCount) - abs(m1Count)) < self.__ACCURACY
                self.__DEBUGGER.debug(abs(abs(targetCount) - abs(m1Count)))
                if m1reached:
                    message += "Stopped M1\n"
                    self.__setReached(motors=[1])

            m0LastCount= m0Count
            m1LastCount = m1Count
            self.__DEBUGGER.debug(message)
            message = ""

        self.stop()
        time.sleep(.5)

    def __RobotRotate(self, pAngle):
        arcRadius = 0.425 #m
        halfArc = arcRadius * math.pi #m
        SPECIAL_K = 1.4
        self.__RobotDrive(halfArc * (pAngle / 180) * 0.5 * SPECIAL_K)

    def forward(self, distance):
        self.__TARGET_MOTORS = [0, 1]
        self.__REVERSE = [1, 1]
        self.__RobotDrive(distance)

    def reverse(self, distance):
        self.__TARGET_MOTORS = [0, 1]
        self.__REVERSE = [1, 1]
        self.__RobotDrive(-distance)

    def right(self, angle, isRadians = False):
    #    self.TARGET_MOTORS = [1]
        self.__REVERSE = [-1, 1]
        if isRadians:
            self.__RobotRotate(angle * (180 / math.pi))
        else:
            self.__RobotRotate(angle)

    def left(self, angle, isRadians = False):
   #     self.TARGET_MOTORS = [0]
        self.__REVERSE = [1, -1]
        if isRadians:
            self.__RobotRotate(angle * (180 / math.pi))
        else:
            self.__RobotRotate(angle)
