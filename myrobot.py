from sr.robot3 import * # type: ignore
import time, math
import util
from datetime import datetime
from pid import PID

class MyRobot:
    __TARGET_MOTORS = [0,1]
    __REVERSE = [1, 1]
    __COUNTS_PER_REVOL = 854.285714286# 897
    __REVOL_DIST = 0.392 # m
    __ACCURACY = 30
    __P, __I, __D = [0.01, 0, 0.00003]
    __ARDUINO_SERIAL = "7523031383335161A231"
    __BAUD = 115200
    ROBOT = None
    __MOTOR_MB = None
    __PUMP_MB = None
    DEBUGGER = None
    __M0FAC = 1
    __M1FAC = 1
    __LAC_MOVE_TIMES = [9, 8]

    def __init__(self, revolDist = 0.392, targetMotors = [0,1], accuracy = 30, dbgEnabled = True, dbgPassThrough = False):
        self.__REVOL_DIST = revolDist
        self.__TARGET_MOTORS = targetMotors
        self.__ACCURACY = accuracy

        self.ROBOT = Robot(raw_ports = [(self.__ARDUINO_SERIAL, self.__BAUD)]) # type: ignore
        self.__MOTOR_MB = self.ROBOT.motor_boards["SR0UK1L"]
        self.__PUMP_MB = self.ROBOT.motor_boards["SR0TJ1P"]

        self.DEBUGGER = util.MyRobotDebug(enable=dbgEnabled, passThrough=dbgPassThrough)
        
        self.scissor_up()

    def __del__(self):
        self.DEBUGGER.stop()

    def stop(self):
        self.__MOTOR_MB.motors[0].power = 0
        self.__MOTOR_MB.motors[1].power = 0

    def __setReached(self, motors = [0,1]):
        if 0 in motors:
            self.__MOTOR_MB.motors[0].power = 0
        if 1 in motors:
            self.__MOTOR_MB.motors[1].power = 0

    def __clamp(self, pmin, x, pmax):
        return max(min(x, pmax), pmin)

    def __powerByCount(self, pCount, pidObject):
        return pidObject(pCount)

    def __pidOverMotors(self, m0Count, m1Count):
        m0Count, m1Count = abs(m0Count), abs(m1Count)
        pDelta = m0Count - m1Count
        pSum = m0Count + m1Count

        if abs(pDelta) < 25:
            return 1, 1
        
        factor = 1 if pSum == 0 else 1 - (abs(pDelta) ** 2) / abs(pSum)
        
        if pDelta > 0:
            return factor, 1
        elif pDelta < 0:
            return 1, factor
        else:
            return 1, 1
        
    def __motorAlign(self, m0Count, m1Count, mIntegral):
        pDelta = m0Count - m1Count
        
        factor = self.__clamp(0, mIntegral / 100, 1)
        
        if m0Count > m1Count:
            return factor, 1
        elif m1Count < m0Count:
            return 1, factor
        else:
            return 1, 1

    def __count_correct(self, mxPID, m0Count, m1Count):
        countDelta = abs(m0Count - m1Count)
        factor = self.__clamp(0, 1 - countDelta / 100, 1);

        print(f"cd: {countDelta}, f: {factor}")

        if m0Count > m1Count:
            self.__M0FAC, self.__M1FAC = factor, 1
        elif m0Count < m1Count:
            self.__M0FAC, self.__M1FAC = 1, factor
        else:
            self.__M0FAC, self.__M1FAC = 1, 1
        
    def __update_facs(self, m0Speed, m1Speed):
        sDelta = m0Speed - m1Speed
        print(sDelta)
        self.__M0FAC -= sDelta / 1000

    def __RobotDrive(self, pDistance, rotate=False):
        self.__M0FAC = 1
        self.__M1FAC = 1

        print(f"Started drive: pDistance: {pDistance}, targets = {self.__TARGET_MOTORS}")
        
        # Calculate target
        targetCount = int(self.__COUNTS_PER_REVOL * pDistance / self.__REVOL_DIST)

        print(f"Going to {targetCount}...\n")

        # Initialise PID objects
        m0PID = PID(self.__P, self.__I, self.__D, setpoint = targetCount * self.__REVERSE[0])
        m0PID.output_limits = (-1,1)
        m1PID = PID(self.__P, self.__I, self.__D, setpoint = targetCount * self.__REVERSE[1])
        m1PID.output_limits = (-1,1)

        mxPID = PID(0.01, 1, 0, setpoint=0)
        mxPID.output_limits = (0,1)

        m0reached = False
        m1reached = False

        m0LastCount = 0
        m1LastCount = 0

        mIntegral = 0

        # Reset motor counts
        self.ROBOT.arduino.command("c")
        initialTime = time.time()
        lastTime = initialTime

        message = ""
        while not m0reached or not m1reached:

            # Get current motor count
            receivedData = self.ROBOT.arduino.command("m")
            message += f"[{time.time() - initialTime}]\n"
            if receivedData[-1] != '|':
                # Badly formatted counts cause problems
                message += f"Bad message received \"{receivedData}\"\n"
                continue
            
            message += f"Received Data: '{receivedData}'\n"

            m0Count, m1Count = list(map(int, receivedData[:-1].split(";")))

            # Get change in count, and set last counts
            m0δ, m1δ = m0Count - m0LastCount, m1Count - m1LastCount
            m0LastCount, m1LastCount = m0Count, m1Count

            mIntegral += abs(m0Count - m1Count)

            # Get time delta
            thisTime = time.time()
            δt = thisTime - lastTime
            lastTime = thisTime

            # Get current motor speed
            m0χ, m1χ = m0δ / δt, m1δ / δt

            if not rotate:
                self.__count_correct(mxPID, m0Count, m1Count)

            # Obtain scaling factors for motor speed
            #factorM0, factorM1 = self.__pidOverMotors(m0Count, m1Count)
            #factorM0, factorM1 = self.__motorAlign(m0Count, m1Count, mIntegral)
            #factorM0, factorM1 = self.__speed_correct(mxPID, m0χ, m1χ)
            #message += f"M0 Factor: {factorM0}\nM1Factor: {factorM1}\n"
            #self.DEBUGGER.debug(f"{factorM0},{factorM1}")

            #self.DEBUGGER.debug(f"{time.time()},{factorM0},{factorM1}")

            if 0 in self.__TARGET_MOTORS and not m0reached:
                self.__MOTOR_MB.motors[0].power = (m0Power := self.__M0FAC * self.__powerByCount(m0Count, m0PID))
                message += f"M0: Count: {m0Count}, M0Fac: {self.__M0FAC}, M0Power; {m0Power}\n"
                m0reached = abs(abs(targetCount) - abs(m0Count)) < self.__ACCURACY and abs(m0χ) < 20
                if m0reached:
                    message += "Stopped M0\n"
                    self.__setReached(motors=[0])

            if  1 in self.__TARGET_MOTORS and not m1reached:
                self.__MOTOR_MB.motors[1].power = (m1Power := self.__M1FAC * self.__powerByCount(m1Count, m1PID))
                message += f"M1: Count: {m1Count}, M1Fac: {self.__M1FAC}, M1Power; {m0Power}\n"
                m1reached = abs(abs(targetCount) - abs(m1Count)) < self.__ACCURACY and abs(m1χ) < 20
                if m1reached:
                    message += "Stopped M1\n"
                    self.__setReached(motors=[1])

            m0LastCount= m0Count
            m1LastCount = m1Count
            print(message)
            #self.DEBUGGER.debug(message)
            message = ""
            time.sleep(0.05)

        self.stop()
     #   self.ROBOT.sleep(.5)

    def __RobotRotate(self, pAngle, SPECIAL_κ):
        self.DEBUGGER.debug(f"Started rotate by {pAngle}...")
        arcRadius = 0.425 # m
        halfArc = arcRadius * math.pi # m

        #SPECIAL_κ = 1.165890625 #1.0127125 # Old: 1.0087125 ## DO NOT CHANGE!

        self.__RobotDrive(halfArc * (pAngle / 180) * 0.5 * SPECIAL_κ, rotate=True)
        
    def __setLacState(self, v):
        self.__PUMP_MB.motors[1].power = v
    
    def __getLacState(self):
        return self.__PUMP_MB.motors[1].power
    
    def __getLacCurrentDraw(self):
        return self.__PUMP_MB.motors[1].current
    
    def getPUMP_MB(self):
        return self.__PUMP_MB
    
    @property
    def pump(self):
        return self.__PUMP_MB.motors[0].power != 0
    
    @pump.setter
    def pump(self, enabled):
        self.__PUMP_MB.motors[0].power = -1 if enabled else 0
    
    def scissor_up(self, dur=-1):
        print("Scissor going up...")
        self.__setLacState(1)
        if dur >= 0:
            self.sleep(dur)
        else:
            while self.__getLacCurrentDraw() < 0.2:
                self.sleep(0.1)
        self.__setLacState(0)
        print("Scissor is up.")
        
    def scissor_down(self, dur=-1):
        print("Scissor going down...")
        self.__setLacState(-1)
        if dur >= 0:
            self.sleep(dur)
        else:
            while self.__getLacCurrentDraw() < 0.2:
                self.sleep(0.1)
        self.__setLacState(0)
        print("Scissor down.")

    def forward(self, distance):
        self.__TARGET_MOTORS = [0, 1]
        self.__REVERSE = [1, 1]
        self.__RobotDrive(distance)

    def reverse(self, distance):
        self.__TARGET_MOTORS = [0, 1]
        self.__REVERSE = [1, 1]
        self.__RobotDrive(-distance)

    def right(self, angle, isRadians = False):
        self.__REVERSE = [-1, 1]
        if isRadians:
            self.__RobotRotate(angle * (180 / math.pi), 1.165890625)
        else:
            self.__RobotRotate(angle, 1.165890625)

    def left(self, angle, isRadians = False):
        self.__REVERSE = [1, -1]
        if isRadians:
            self.__RobotRotate(angle * (180 / math.pi), 1.29725)
        else:
            self.__RobotRotate(angle, 1.29725)
    
    def grab(self):
        self.pump = True
        self.scissor_down()
        self.sleep(1)
        self.scissor_up()
    
    def drop(self):
        self.pump = False
    
    def place(self):
        self.scissor_down()
        self.pump = False
        self.scissor_up()
    
    def beep(self, hz=440, dur=0.1):
        self.ROBOT.power_board.piezo.buzz(hz, dur)
    
    def beep_sync(self, hz=440, dur=0.1, timeout=0):
        self.ROBOT.power_board.piezo.buzz(hz, dur, blocking=True)
        if timeout > 0:
            self.ROBOT.sleep(timeout)
    
    def sleep(self, length):
        self.ROBOT.sleep(length)

    def see_and_capture(self, f):
        return self.ROBOT.camera.see(save=f)