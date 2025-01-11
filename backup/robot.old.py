from sr.robot3 import *
import time

ARDUINO_SERIAL = "7523031383335161A231"
BAUD = 115200
robot = Robot(raw_ports = [(ARDUINO_SERIAL, BAUD)])
mb = robot.motor_board
mb.motors[0].power = 0.75
mb.motors[1].power = -0.75


numberOfBytes = 8

while True:
    receivedData = robot.arduino.command("m")
    M0 = receivedData[0:4][::-1]
    M1 = receivedData[4:][::-1]
    
    print(sum([ord(M0[i]) *256** (3-i) for i in range(0,4)]), sum([ord(M1[i]) * 256** (3-i) for i in range(0,4)]))
    time.sleep(1)
    
    
    #int values[4] = {0x12, 0x34, 0x56, 0x78};
    #int final = 0;
    #for (int i = 0; i < 4; i++)
    #{
    #   final |= value[i] << (i * 8);
    #}


