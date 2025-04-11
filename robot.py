from myrobot import *
import sys
import unittest

# Uncomment the test you want to run.
# Tests are defined as functions in tests.py

TESTING = True
tests_to_run = [
    #"drive_back_forwards",
    #"lots_of_rotation_left",
    #"lots_of_rotation_right",
    "rotate_20",
    #"drive_back_forwards",
    #"drive_back_forwards",
    #"navigate_to_cube",
    #"drive_right_triangle",
    # "lac_up_down",
    # "lac_time_up_down",
    # "pump_on_off",
    # "stand_and_deliver",
    #"full_scissor_test",
    # "pump_read_current",
    # "scissor_read_current",
    # "rickroll",
    # "motor_challenge",
    # "play_fetch",
    # "pump_toggler"
    #"back_forwards_12_billion",
    # "up_down_12_billion",
    #"game_tests"
]

if TESTING:
    import tests #Avoid double robot declaration by importing here
    unittest.main(module=tests.TestRobot, verbosity=2, defaultTest=tests_to_run)
    # unittest.main exists after completion

robot = MyRobot(accuracy=10, dbgEnabled=False)


sys.exit(0)