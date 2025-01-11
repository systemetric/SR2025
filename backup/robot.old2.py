from sr.robot3 import *
import time
from datetime import datetime
from pid import PID

ARDUINO_SERIAL = "7523031383335161A231"
BAUD = 115200
robot = Robot(raw_ports = [(ARDUINO_SERIAL, BAUD)])
mb = robot.motor_board

#R, L = 0, 1
numberOfBytes = 8

COUNTS_PER_REVOL = 2720

NUMBER_OF_REVOL = 10
distance10Revols = 3.92 #m
loops = 10
FACTOR = 10
INITIAL = FACTOR * int(COUNTS_PER_REVOL * NUMBER_OF_REVOL / distance10Revols)
TARGET = INITIAL
WINDOW = 200

THRESHOLD = 0.01

vals = [0.01, 0.1, 1, 10, 100]

def f(x):
    return x
    
def clamp(val, min, max):
    if (val < min):
        return min
    elif (val > max):
        return max
    else:
        return val
    
def powerByCount(pCount, pMotor, f):
    if pCount < TARGET - WINDOW:
     #   print(f"Motor {pMotor}: Count: {pCount}, Motor Power: 1")
        f.write(f"Motor {pMotor}: Count: {pCount}, Motor Power: 1\n")
        return 1.0
    
    remaining = TARGET - pCount
    res = clamp(remaining / WINDOW, -1, 1)
    #print(f"Motor {pMotor}: Count: {pCount}, Motor Power: {res}")
    f.write(f"Motor {pMotor}: Count: {pCount}, Motor Power: {res}\n")
    
    return res
    
t = time.time()
time.sleep(3)
for _ in range(loops):
    robot.arduino.command("c")
    initial = time.time()
    d = datetime.now()
    z = d.strftime("%d/%m/%Y_%H:%M:%S")
    f = open(f"log_{z}.txt", "w")
    while time.time() - initial < 60:
        f.write(f"[{time.time() - initial}]\n")
        receivedData = robot.arduino.command("m")
    #    print(f"Received Data: '{receivedData}'")
        f.write(f"Received Data: '{receivedData}'\n")
        
    #    for _ in range(4):
    #        print()
            
        receivedDataSplit = receivedData.split(";")
        
        if receivedData[-1] != '|':
    #        print(f"Bad message received \"{receivedData}\"")
            f.write(f"Bad message received \"{receivedData}\"\n")
        else:
            
            M0 = int(receivedDataSplit[0])
            mb.motors[0].power = powerByCount(M0,0,f)

            M1 = int(receivedDataSplit[1][:-1])
            mb.motors[1].power = powerByCount(M1,1,f)
    f.close()
        
    
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
