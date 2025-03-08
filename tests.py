import unittest
import time
from myrobot import *
from camera import *

robot = MyRobot(accuracy=10, dbgEnabled=True)

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
        robot.forward(1.5)
        robot.right(180)
        robot.forward(1.5)
        robot.right(180)

    def drive_right_triangle(self):
        robot.forward(1.23)
        robot.right(137)
        robot.forward(1.4 * (2**0.5))
        robot.right(137)
        robot.forward(1.47)
        robot.right(91)
    
    def motor_challenge(self):
        for _ in range(3):
            self.drive_right_triangle()

    def lac_up_down(self):
        robot.scissor_up()
        robot.sleep(2)
        robot.scissor_down()
    
    def lac_time_up_down(self):
        for _ in range(3):
            robot.beep()
            robot.sleep(1)
        robot.beep(880)
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
        robot.pump = True
        robot.sleep(10)
        robot.pump = False
    
    def full_scissor_test(self):
        robot.grab()
        robot.sleep(0.4)
        robot.beep_sync(440, 0.15, 0.05)
        robot.sleep(0.4)
        robot.drop()
    
    # WILL CRASH
    def scissor_read_current(self):
        robot.d__setLacState(-1)
        print("time, current")
        for i in range(200):
            print(f"{i*0.1}, {robot.pumpmb().motors[1].current}")
            robot.sleep(0.1)
        robot.d__setLacState(0)
        
    def pump_read_current(self):
        robot.pump = True
        print("time, current")
        for i in range(300):
            print(f"{i*0.1}, {robot.getPUMP_MB().motors[0].current}")
            robot.sleep(0.1)
        robot.pump = False
    
    def stand_and_deliver(self):
        robot.forward(1)
        robot.grab()
        robot.right(180)
        robot.forward(1)
        robot.drop()
        robot.right(180)
    
    def play_fetch(self):
        print("Playing fetch")
        while True:
            robot.forward(1)
            robot.grab()
            robot.right(180)
            robot.forward(1)
            robot.drop()
            robot.right(180)

            robot.forward(1)
            robot.right(180)
            robot.forward(1)
            robot.right(180)
        
    def rickroll(self):
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
        robot.beep_sync(312, 0.55,0.05) #Eish
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(262, 0.15, 0.05) #C
        
    def pump_toggler(self):
        for _ in range(10):
            robot.pump = True
            robot.sleep(3)
            robot.pump = False
            robot.sleep(2)
    
    def navigate_to_cube(self):
        mrc = MyRobotCamera(robot)

        for i in range(36):
            robot.right(20)
            m = mrc.find_pallet_markers()

            if len(m) > 0:
                print(f"{i}:")
                for mx in m:
                    print(f"    ID = {mx.id}    Distance = {mx.position.distance}    Angle = {mx.position.horizontal_angle}")

                closest = m[0]
                robot.right(closest.position.horizontal_angle, isRadians=True)
                robot.forward((closest.position.distance) / 1000)
                robot.grab()

if __name__ == '__main__':
    unittest.main()