from myrobot import *
import time, math

robot = MyRobot(accuracy=25, dbgEnabled=False)

for i in range(10):
    robot.right(90)
    time.sleep(2)
    robot.left(90)