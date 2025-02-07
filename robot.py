from myrobot import *
import time, math, sys

robot = MyRobot(accuracy=10, dbgEnabled=True)

"""
for i in range(5):
    robot.right(360)

sys.exit(0)
"""

"""
for i in range(5):
    robot.right(3600)
    time.sleep(10)
"""


for _ in range(3):
    robot.forward(1.5)
    robot.right(90)
    robot.forward(1.5)
    robot.right(135)
    robot.forward(1.5 * (2) ** (1/2))
    robot.right(135)

sys.exit(0)


"""
for i in range(4):
    robot.right(90)
    time.sleep(2)
    robot.left(90)
    time.sleep(2)
"""