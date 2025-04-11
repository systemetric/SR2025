from myrobot import *
import sys
import unittest
import enum

# Uncomment the test you want to run.
# Tests are defined as functions in tests.py

TESTING = False
tests_to_run = [
    #"drive_back_forwards",
    #"lots_of_rotation_left",
    #"lots_of_rotation_right",
    "rotate_20",
    #"drive_reverse",
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

TARGET_HIGHRISE = 195 + robot.zone
print("Hello! I'm looking for high rise:", TARGET_HIGHRISE)

runningCompetition = True
visitedMarkers = []

class RobotState(enum.Enum):
    LOOKING_FOR_CUBES = enum.auto()
    GRABBING = enum.auto()
    LOOKING_FOR_DISTRICT = enum.auto()
    PLACE = enum.auto()

state = RobotState.LOOKING_FOR_CUBES
total_search_rotation = 0

while runningCompetition:
    print("Current state:", state)

    if (state == RobotState.LOOKING_FOR_CUBES):
        # find a pallet marker to navigate to
        # |  || || |_
        pallet_markers = robot.camera.find_pallet_markers(ignored_markers=visitedMarkers)
        
        # if we have turned all the way around, not found anything, reverse.
        if total_search_rotation > 360:
            robot.reverse(0.5)
            total_search_rotation = 0

        if len(pallet_markers) == 0:
            robot.right(20)
            total_search_rotation += 20
        else:
            total_search_rotation = 0

            pallet = pallet_markers[0]
            print("Going to", pallet)

            robot.right(pallet.position.horizontal_angle, isRadians=True)
            robot.forward(((pallet.position.distance) / 1000) - 0.2)

            print("I think I've arrived at", pallet)
            state = RobotState.GRABBING

    elif (state == RobotState.GRABBING):
        print("Grabbing pallet...")
        robot.grab()

        if (robot.pump_grabbing_noise_based()):
            visitedMarkers.append(pallet)
            print("Grabbed pallet :)")
            state = RobotState.LOOKING_FOR_DISTRICT
        else:
            print("Cube grab failed. Restarting...")
            robot.drop()
            robot.reverse(0.5)
            state = RobotState.LOOKING_FOR_CUBES
    
    elif (state == RobotState.LOOKING_FOR_DISTRICT):
        markers = robot.camera.find_all_markers()

        plinth = None
        for marker in markers:
            if marker.id == TARGET_HIGHRISE:
                plinth = marker
                break

        if robot.camera.is_pallet_on_plinth(markers, TARGET_HIGHRISE):
            # get high rise marker info
            for marker in markers:
                if marker.id == TARGET_HIGHRISE:
                    highrise = marker
                    break

            robot.right(highrise.position.horizontal_angle, isRadians=True)
            robot.forward(((highrise.position.distance) / 1000) - 0.2)
            state = RobotState.PLACE
        
        elif plinth:
            robot.right(plinth.position.horizontal_angle, isRadians=True)
            robot.forward(((plinth.position.distance) / 1000))
            state = RobotState.PLACE
        else:
            target = None
            for marker in markers:
                if marker.id in visitedMarkers:
                    target = marker
                    break
            
            if not target:
                robot.right(15)
            else:
                robot.right(target.position.horizontal_angle, isRadians=True)
                robot.forward(((target.position.distance) / 1000))
                state = RobotState.PLACE
    
    elif (state == RobotState.PLACE):
        robot.place()
        robot.reverse(1)
        state = RobotState.LOOKING_FOR_CUBES

sys.exit(0)