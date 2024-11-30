from sr.robot3 import *
import time
from pid import PID

ARDUINO_SERIAL = "7523031383335161A231"
BAUD = 115200
robot = Robot(raw_ports = [(ARDUINO_SERIAL, BAUD)])
mb = robot.motor_board
#mb.motors[1].power = 1

numberOfBytes = 8

COUNTS_PER_REVOL = 1575

TARGET = 16000
WINDOW = 200

THRESHOLD = 0.01

countThreshold = 30
countsPerSecondThreshold = 30
outputLimit = 1000

vals = [0.01, 0.1, 1, 10, 100]

def f(x):
    return x
def powerByCount(pCount):

    if pCount < TARGET - WINDOW:
        return 1.0
    
    remaining = TARGET - pCount
    res = remaining/WINDOW
    print(f"Count: {pCount}, Motor Power: {res}")
    
    return res
    
t = time.time()
prevM0 = 0
while True:
    
    receivedData = robot.arduino.command("m")
    
    if receivedData[-1] != '|':
        print(f"Bad message received \"{receivedData}\"")
    else:
        M0 = int(receivedData.split(";")[0])
        mb.motors[0].power = powerByCount(M0)
        
        
        
        if (abs(M0 - TARGET) < countThreshold) and ((M0 - prevM0)/(time.time() - t) < countsPerSecondThreshold):
            print(M0)
            break

    prevM0 = M0
    t = time.time()
    
print("Exited.")
    

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
