from myrobot import *
import sys
import unittest

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
    import tests #Avoid double robot declaration by importing here
    unittest.main(module=tests.TestRobot, verbosity=2, defaultTest=tests_to_run)
    # unittest.main exists after completion

robot = MyRobot(accuracy=10, dbgEnabled=True)

robot.__PUMP_MB.motors[1].power = 1
robot.sleep(5)
robot.__PUMP_MB.motors[1].power = 0
robot.sleep(2)
robot.__PUMP_MB.motors[1].power = -1
robot.sleep(5)
robot.__PUMP_MB.motors[1].power = 0

sys.exit(0)