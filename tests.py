import unittest
import time
from myrobot import *

robot = MyRobot(accuracy=10, dbgEnabled=True)

class TestRobot(unittest.TestCase):        
    def drive_back_forwards(self):
        robot.forward(1.5)
        robot.right(180)
        robot.forward(1.5)
        robot.right(180)

    def drive_right_triangle(self):
        robot.forward(1.5)
        robot.right(135)
        robot.forward(1.5 * (2**0.5))
        robot.right(135)
        robot.forward(1.5)
        robot.right(90)

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
        
    def rickroll(self):
        robot.beep_sync(262, 0.15, 0.05) #C
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(349, 0.15, 0.05) #F
        robot.beep_sync(294, 0.15, 0.05) #D
        robot.beep_sync(440, 0.55, 0.05) #A
        robot.beep_sync(440, 0.55, 0.05) #A
        robot.beep_sync(392, 1.1,0.1) #A
        
        robot.beep_sync(262, 0.2) #C
        robot.beep_sync(294, 0.2) #D
        robot.beep_sync(349, 0.2) #F
        robot.beep_sync(294, 0.2) #D
        robot.beep_sync(392, 0.55, 0.05) #A
        robot.beep_sync(392, 0.55, 0.05) #A
        robot.beep_sync(349, 1.1,0.1) #A
        
        
        
        
    
    

if __name__ == '__main__':
    unittest.main()