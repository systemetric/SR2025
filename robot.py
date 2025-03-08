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
    # "stand_and_deliver",
    # "full_scissor_test",
    "pump_read_current",
    # "scissor_read_current",
    # "rickroll",
    # "motor_challenge",
]

if TESTING:
    import tests #Avoid double robot declaration by importing here
    unittest.main(module=tests.TestRobot, verbosity=2, defaultTest=tests_to_run)
    # unittest.main exists after completion

robot = MyRobot(accuracy=10, dbgEnabled=True)


sys.exit(0)