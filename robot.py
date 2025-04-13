from myrobot import *
import sys
import unittest
import enum

# Uncomment the test you want to run.
# Tests are defined as functions in tests.py

TESTING = False
tests_to_run = [
    # "drive_for_scores"
    # "drive_back_forwards",
    # "lots_of_rotation_left",
    # "lots_of_rotation_right",
    # "rotate_20",
    # "drive_full",
    # "drive_reverse",
    # "drive_back_forwards",
    # "drive_back_forwards",
    # "navigate_to_cube",
    # "drive_right_triangle",
    # "lac_up_down",
    # "lac_time_up_down",
    # "pump_on_off",
    # "stand_and_deliver",
    # "full_scissor_test",
    # "pump_read_current",
    "scissor_read_current",
    # "rickroll",
    # "motor_challenge",
    # "play_fetch",
    # "pump_toggler"
    # "back_forwards_12_billion",
    # "up_down_12_billion",
    # "game_tests",
    # "log_lac_values"
]

if TESTING:
    import tests  # Avoid double robot declaration by importing here

    unittest.main(module=tests.TestRobot, verbosity=2, defaultTest=tests_to_run)
    # unittest.main exists after completion

# LAC Stop smart **
# LAC Async **
# LAC Part lift
# Reduce reverse distance **
# Reduce threshold for slow drive **
# Time game round for centre knock **

robot = MyRobot(accuracy=40, dbgEnabled=False)

TARGET_HIGHRISE = 195 + robot.zone
print("Hello world! I'm looking for high rise:", TARGET_HIGHRISE)

runningCompetition = True
delivered_markers = []
turn_dir = 1


class RobotState(enum.Enum):
    LOOKING_FOR_CUBES = enum.auto()
    GRABBING = enum.auto()
    LOOKING_FOR_DISTRICT = enum.auto()
    PLACE = enum.auto()
    
robot.forward(1)
# after we've driven, wait for the arm to finish raising.
# robot.await_scissor_complete()

state = RobotState.LOOKING_FOR_CUBES
total_search_rotation = 0
while runningCompetition:
    print("Current state:", state)
    print("Seconds left:", robot.get_seconds_remaining())
    print("Delivered Markers", delivered_markers)

    if state == RobotState.LOOKING_FOR_CUBES:
        # find a pallet marker to navigate to
        # |  || || |_
        markers: list = robot.camera.find_all_markers()
        print("Markers:", markers)

        if len(markers) == 0:
            print("No markers found. Reversing 1m.")
            robot.reverse(1)
            total_search_rotation = 0
            continue

        markers = filter(lambda x: x.id not in delivered_markers, markers)

        pallet_markers = robot.camera.check_if_markers_are_our_pallets(
            markers, ignored_markers=delivered_markers
        )

        # if we have turned all the way around, not found anything, reverse.
        if total_search_rotation > 360:
            print("No pallets we are interested in could be found, reversing.")
            robot.reverse(0.5)
            total_search_rotation = 0

        if len(pallet_markers) == 0:
            robot.right(20)
            total_search_rotation += 20
        else:
            print("Pallet found.")
            total_search_rotation = 0

            pallet = pallet_markers[0]
            print("Going to", pallet)

            driven_to_cube = robot.drive_to(pallet)

            if driven_to_cube:
                print("I think I've arrived at", pallet)
                state = RobotState.GRABBING
            else:
                print("Not arrived at cube.")

    elif state == RobotState.GRABBING:
        print("Grabbing pallet...")
        robot.async_grab_carefully()

        if robot.pump_grabbing_noise_based():
            delivered_markers.append(pallet.id)
            print("Grabbed pallet :)")
            robot.reverse(0.5)
            state = RobotState.LOOKING_FOR_DISTRICT
        else:
            print("Cube grab failed. Restarting...")
            robot.drop()
            robot.reverse(0.5)
            state = RobotState.LOOKING_FOR_CUBES
        
        '''
        *** HOTEL ROOM TESTING CODE: ***  
        Swap the below with the above! 
        
        robot.reverse(0.5)
        delivered_markers.append(pallet.id)
        print(
            "Added pallet to visited markers. Not checking for pallet actually grabbed!"
        )
        state = RobotState.LOOKING_FOR_DISTRICT'''

    elif state == RobotState.LOOKING_FOR_DISTRICT:
        # camera find all markers
        markers = robot.camera.find_all_markers()

        # look for our high rise
        plinth = None
        for marker in markers:
            if marker.id == TARGET_HIGHRISE:
                plinth = marker
                break

        print("Plinth found:", plinth)

        pallet_on_plinth = robot.camera.is_pallet_on_plinth(markers, TARGET_HIGHRISE)

        if plinth and pallet_on_plinth:
            print("Stacked plinth")
            if not robot.drive_to(plinth, minus=0.2):
                print("Couldn't find plinth")
                robot.right(30)
                continue
            state = RobotState.PLACE
        elif plinth:
            print("Clear plinth")
            if not robot.drive_to(plinth):
                print("Couldn't find plinth")
                robot.right(30)
                continue
            state = RobotState.PLACE
        else:
            print("General area")
            # go to a marker we've already moved (it must be near the highrise as we've moved it previously.)
            target = None
            for marker in markers:
                if marker.id in delivered_markers:
                    target = marker
                    break

            # turn if nothing found
            if not target:
                robot.right(20 * turn_dir)
            else:
                # go to target place cube
                if not robot.drive_to(target):
                    print("Couldn't find plinth")
                    robot.right(30)
                    continue
                state = RobotState.PLACE

    elif state == RobotState.PLACE:
        robot.async_place()
        # robot.async_place()
        robot.reverse(0.5)
        turn_dir = -1
        state = RobotState.LOOKING_FOR_CUBES

sys.exit(0)
