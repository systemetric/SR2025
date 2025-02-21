import unittest
import time
from myrobot import *

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
        
    def rickroll(self):
        robot.beep_sync(262, 0.15, 0.05) #C
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(349, 0.15, 0.05) #F
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(440, 0.55, 0.05) #A
        robot.beep_sync(440, 0.55, 0.05) #A
        robot.beep_sync(392, 1.1,0.1) #G
        
        robot.beep_sync(262, 0.2) #C
        robot.beep_sync(294, 0.2) #D
        robot.beep_sync(349, 0.2) #F
        robot.beep_sync(294, 0.2) #D
        robot.beep_sync(392, 0.55, 0.05) #G
        robot.beep_sync(392, 0.55, 0.05) #G
        robot.beep_sync(349, 1.1,0.1) #F
        
        
        
        
    
    

if __name__ == '__main__':
    unittest.main()