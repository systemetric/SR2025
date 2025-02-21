from myrobot import *
import sys
import unittest
import tests

TESTING = True
tests_to_run = [
    # "drive_back_forwards",
    # "drive_right_triangle",
    # "lac_up_down",
    # "lac_time_up_down",
    # "pump_on_off",
    "rickroll",
]

if TESTING:
    unittest.main(module=tests.TestRobot, verbosity=2, defaultTest=tests_to_run)

robot = MyRobot(accuracy=10, dbgEnabled=True)

"""
for i in range(5):
    robot.right(360)

sys.exit(0)
"""

"""
for i in range(5):
    robot.right(3600)
    robot.sleep(10)
"""


# for _ in range(3):
#     robot.forward(1.5)
#     robot.right(180)
#     robot.forward(1.5)
#     robot.right(180)

robot.__PUMP_MB.motors[1].power = 1
robot.sleep(5)
robot.__PUMP_MB.motors[1].power = 0
robot.sleep(2)
robot.__PUMP_MB.motors[1].power = -1
robot.sleep(5)
robot.__PUMP_MB.motors[1].power = 0

sys.exit(0)


"""
for i in range(4):
    robot.right(90)
    robot.sleep(2)
    robot.left(90)
    robot.sleep(2)
"""