from sr.robot3 import *
import time

robot = Robot()
baseDelay = 0.005

for multiple in range(1):
    robot.kch.leds[LED_C].colour = Colour.OFF
    print(f"The current delay is {multiple * baseDelay}.")
    for count in range(13000):
        print(f"The current count is: {count}")
        time.sleep(0.001)

    
    robot.kch.leds[LED_C].colour = Colour.RED
    time.sleep(1)
    
robot.kch.leds[LED_C].colour = Colour.BLUE
print(f"This is done at {time.time()}.")