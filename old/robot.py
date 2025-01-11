from sr.robot3 import *
import time
from datetime import datetime
from pid import PID
import math

ARDUINO_SERIAL = "7523031383335161A231"
BAUD = 115200
robot = Robot(raw_ports = [(ARDUINO_SERIAL, BAUD)])
mb = robot.motor_board

#R, L = 0, 1

COUNTS_PER_REVOL = 2720

DISTANCE = 3
revolDistance = 0.392 #m
loops = 10

INITIAL = 0
TARGET = 0
WINDOW = 200

DMOTOR0 = 0
DMOTOR1 = 1

THRESHOLD = 0.01

DEBUG = True
DEBUG_TIME = datetime.now()
DEBUG_TIMES = DEBUG_TIME.strftime("%d.%m.%Y_%H.%M.%S")
DEBUG_FILE = open(f"log_{DEBUG_TIMES}.txt", "w")

def f(x):
    return x
    
def powerByCount(pCount, pMotor, pTarget):
    if pCount < pTarget - WINDOW:
        debug(f"Motor {pMotor}: Count: {pCount}, Motor Power: 1\n")
        return 1.0
    
    remaining = pTarget - pCount
    res = clamp(remaining / WINDOW, -1, 1)
    debug(f"Motor {pMotor}: Count: {pCount}, Motor Power: {res}\n")
    
    return res
    
"""
t = time.time()
time.sleep(3)
for _ in range(loops):
    robot.arduino.command("c")
    initial = time.time()
    while time.time() - initial < 60:
        debug(f"[{time.time() - initial}]")
        receivedData = robot.arduino.command("m")
        debug(f"Received Data: '{receivedData}'")
            
        receivedDataSplit = receivedData.split(";")
        
        if receivedData[-1] != '|':
            debug(f"Bad message received \"{receivedData}\"")
        else:
            
            M0 = int(receivedDataSplit[0])
            mb.motors[0].power = powerByCount(M0,0,f)

            M1 = int(receivedDataSplit[1][:-1])
            mb.motors[1].power = powerByCount(M1,1,f)
        
debug("Exited.")
DEBUG_FILE.close()
"""


def RobotDrive(distance, dm = 2):
    targetCount = int(COUNTS_PER_REVOL * distance / revolDistance)
    
    robot.arduino.command("c")
    initialTime = time.time()

    while time.time() - initialTime < 60:
        debug(f"[{time.time() - initialTime}]")
        receivedData = robot.arduino.command("m")
        debug(f"Received Data: '{receivedData}'")
            
        receivedDataSplit = receivedData.split(";")
        
        if receivedData[-1] != '|':
            debug(f"Bad message received \"{receivedData}\"")
        else:
            
            if (dm == 0 or dm == 2):
                M0 = int(receivedDataSplit[0])
                mb.motors[0].power = powerByCount(M0,0, targetCount)

            if (dm == 1 or dm == 2):
                M1 = int(receivedDataSplit[1][:-1])
                mb.motors[1].power = powerByCount(M1,1, targetCount)

def RobotRotate(a, m):
    RobotDrive(((0.425 * math.pi) / 180) * a, m)

def ExitDebug():
    DEBUG_FILE.close()

## PUT DRIVING CODE HERE ##
debug("Started")


RobotDrive(1)
RobotRotate(90, DMOTOR0)

##    END DRIVING CODE   ##

ExitDebug()


"""


for p in range(4):
    for i in range(4):
        for d in range(4):
            print(f"Start test {p} {i} {d}...")
            mb.motors[0].power = 1
            robot.arduino.command('c')
            pidM0 = PID(vals[p], vals[i], vals[d], setpoint=TARGET)
            pidM0.output_limits = (-outputLimit,outputLimit)
            startTime = time.time()
            currentTime = time.time()
            while currentTime - startTime > 5:
                receivedData = robot.arduino.command("m")
                
                if receivedData[-1] != '|':
                    print(f"Bad message received \"{receivedData}\"")
                else:
                    M0 = int(receivedData.split(";")[0])
                    mb.motors[0].power = pidM0(M0) / outputLimit
                    
                    print(M0)
                    
               #     M1 = int(receivedData.split(";")[1][:-1])
                #    mb.motors[0].power = powerByCount(M0)
                 #   mb.motors[1].power = powerByCount(M1)
                  #   
                   
                time.sleep(0.01)
                currentTime = time.time();
            mb.motors[0].power = 0;
            time.sleep(2);
    
    #int values[4] = {0x12, 0x34, 0x56, 0x78};
    #int final = 0;
    #for (int i = 0; i < 4; i++)
    #{
    #   final |= value[i] << (i * 8);
    #}

"""
