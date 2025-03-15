import unittest
import time
from myrobot import *
from camera import *

robot = MyRobot(accuracy=25, dbgEnabled=True)

class TestRobot(unittest.TestCase):
    def setUp(self):
        robot.sleep(1)
        for _ in range(3):
            robot.beep_sync(440, 0.8, 0.2)
        robot.beep_sync(880, 1,0.2)
    
    def tearDown(self):
        robot.sleep(1)
        robot.beep_sync(880, 0.1, 0.05) #A
        robot.beep_sync(880, 0.3, 0.05) #A
        robot.sleep(1)
          
    def drive_back_forwards(self):
        """Drive forwards, turn around, drive backwards.
        """
        robot.forward(3)
       # robot.right(180)
      #  robot.forward(3)
       # robot.right(180)

    def drive_right_triangle(self):
        """Drive in a triangle, turns right each time.
        """
        robot.forward(1.5)
        robot.right(145)
        robot.forward(1.5 * (2**0.5))
        robot.right(145)
        robot.forward(1.5)
        robot.right(90)
    
    def motor_challenge(self):
        """Drives in the right triangle three times.
        """
        for _ in range(3):
            self.drive_right_triangle()

    def lac_up_down(self):
        """Move the scissor lift up and down
        """
        robot.scissor_up()
        robot.sleep(2)
        robot.scissor_down()
    
    def lac_time_up_down(self):
        """Legacy method for moving the linear actuator based on time.
        """
        robot.scissor_up(12)
        for _ in range(3):
            robot.beep(220)
            robot.sleep(0.5)
            
        robot.sleep(5)
        
        for _ in range(3):
            robot.beep()
            robot.sleep(1)
        robot.beep(880)
        robot.scissor_down(12)
        for _ in range(3):
            robot.beep(220)
            robot.sleep(0.5)
    
    def pump_on_off(self):
        """Turn on pump for ten seconds then turn it off.
        """
        robot.pump = True
        robot.sleep(10)
        robot.pump = False
    
    def full_scissor_test(self):
        """Pick up a cube that's under the grabber, then drop it.
        """
        robot.grab()
        robot.sleep(0.4)
        robot.beep_sync(440, 0.15, 0.05)
        robot.sleep(0.4)
        robot.drop()
    
    def up_down_12_billion(self):
        for _ in range(100):
            robot.scissor_down()
            robot.sleep(0.1)
            robot.scissor_up()
            robot.sleep(0.1)
            if len(robot.see_and_capture()) != 0:
                break;
    
    def back_forwards_12_billion(self):
        while len(robot.see_and_capture("beans.png")) == 0:
            robot.forward(2)
            robot.right(180)
            robot.forward(2)
            robot.left(180)
    
    def pump_grab_noise_based(self):
        robot.pump = True
        for _ in range(20):
            robot.beep_sync(262, 0.15, 0.05) #C
            robot.beep_sync(392, 0.55, 0.05) #G
            robot.sleep(0.5)
            print("Checking...")
            if robot.pump_grabbing_noise_based():
                print("Grabbed!")
                robot.sleep(0.5)
                robot.beep_sync(880, 0.1, 0.05) #A
                robot.beep_sync(880, 0.3, 0.05) #A
            else:
                print("Not Grabbed!")
                robot.sleep(0.5)
                robot.beep_sync(262, 0.15, 0.05) #C
            robot.sleep(5)
            
    # WILL CRASH
    def scissor_read_current(self):
        """Stream current from linear actuator.
        """
        robot.d__setLacState(-1)
        print("time, current")
        for i in range(200):
            print(f"{i*0.1}, {robot.pumpmb().motors[1].current}")
            robot.sleep(0.1)
        robot.d__setLacState(0)
        
    def pump_read_current(self):
        """Stream current from pump.
        """
        robot.pump = True
        print("time, current")
        for i in range(300):
            print(f"{i*0.1}, {robot.getPUMP_MB().motors[0].current}")
            robot.sleep(0.1)
        robot.pump = False
    
    def stand_and_deliver(self):
        """Go forward, 1m grab a cube, turn and go back 1m, drop it.
        """
        robot.forward(1)
        robot.grab()
        robot.right(180)
        robot.forward(1)
        robot.drop()
        robot.right(180)
    
    def play_fetch(self):
        """Tests calibration of motors and grbaber.
        Picks up cube, goes forward 1m then drops it.
        Turn around drive 1m, turn around drive 1m.
        Should be back at where we started and able to pick up the cube again.
        """
        print("Playing fetch")
        while True:
            robot.grab()
            robot.forward(1)
            robot.drop()
            robot.right(180)
            robot.forward(1)
            robot.right(180)
            robot.forward(1)
        
    def rickroll(self):
        """Do I need to explain what this does? Try it.
        """
        robot.beep_sync(262, 0.15, 0.05) #C
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(349, 0.15, 0.05) #F
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(440, 0.55, 0.05) #A
        robot.beep_sync(440, 0.55, 0.05) #A
        robot.beep_sync(392, 1.1,0.1) #G
        
        robot.beep_sync(262, 0.15, 0.05) #C
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(349, 0.15, 0.05) #F
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(392, 0.55, 0.05) #G
        robot.beep_sync(392, 0.55, 0.05) #G
        robot.beep_sync(349, 1.1,0.1) #F
        
        robot.beep_sync(262, 0.15, 0.05) #C
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(349, 0.15, 0.05) #F
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(349, 0.55, 0.05) #F
        robot.beep_sync(392, 0.55, 0.05) #G
        robot.beep_sync(324, 0.55,0.05) #Eish
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(262, 0.4, 0.4) #C
        
        robot.beep_sync(262, 0.35, 0.05) #C
        robot.beep_sync(392, 0.75, 0.05) #G
        robot.beep_sync(349, 1.1,0.1) #F
        
        
    def pump_toggler(self):
        """Tests suction and release in pipes.
        Turns the suction pump on and off 10 times.
        """
        for _ in range(10):
            robot.pump = True
            robot.sleep(3)
            robot.pump = False
            robot.sleep(2)
    
    def dist_func(self, x):
        κ1 = 0.03026
        κ2 = 1.00968
        κ3 = 0.01906

        return (((-κ2) + math.sqrt(κ2**2 - 4 * κ3 * (κ1 - x/1000))) / (2 * κ3))

    def pick_up_cube(self, mrc, i, g):
        for i in range(18):
            robot.right(20)

            m = []
            if i == 0:
                m = mrc.find_pallet_markers()
            elif i == 1:
                m = mrc.find_high_rise_markers()

            if len(m) > 0:
                for i in range(4):
                    if i == 0:
                        m = mrc.find_pallet_markers()
                    elif i == 1:
                        m = mrc.find_high_rise_markers()

                    if len(m) > 0:
                        print(f"{i}:")
                        for mx in m:
                            print(f"    ID = {mx.id}    Distance = {mx.position.distance}    Angle = {mx.position.horizontal_angle}")

                        closest = m[0]
                        robot.right(closest.position.horizontal_angle, isRadians=True)
                        robot.forward(self.dist_func(closest.position.distance) / (4 - i))

                if g == True:
                    robot.grab()
                else:
                    robot.drop()

    def navigate_to_cube(self):
        """Uses camera to locate a cube, drive to it and pick it up.
        """
        mrc = MyRobotCamera(robot)

        while True:
            self.pick_up_cube(mrc, 0, True)
            self.pick_up_cube(mrc, 1, False)

    def go_to_cube(self, m):
        for i in range(4):
            robot.right(m.position.horizontal_angle, isRadians=True)
            robot.forward(self.dist_func(m.position.distance) / (4 - i))

    def game_tests(self):
        mrc = MyRobotCamera(robot)
        robot.forward(1200)
        running = True
        found = False
        while running:
            if not found:
                markers = mrc.find_pallet_markers()
                if len(markers) > 0:
                    self.go_to_cube(mrc, markers[0])
                    robot.grab()
                    found = True
                robot.right(10)        
            else:
                markers = mrc.find_high_rise_markers()
                if len(markers) > 0:
                    self.go_to_cube(mrc, markers[0])
                    robot.drop()
                    found = False
                robot.left(10)


if __name__ == '__main__':
    unittest.main()