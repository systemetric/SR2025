from myrobot import *
import time

robot = MyRobot(accuracy=25, dbgEnabled=True)

for i in range(12):
    robot.forward(i % 3 + 1)
    time.sleep(2)
    robot.reverse(i % 3 + 1)
    time.sleep(2)