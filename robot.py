import os,sys,math,time

current_directory = os.getcwd()
parent_directory = os.path.dirname(current_directory)
sys.path.insert(0, f"{parent_directory}/myrobot")
sys.path.insert(0, f"{parent_directory}")

from myrobot import *


robot = MyRobot(dbgEnabled=False)
srRobot = robot.ROBOT
got_marker = False

print("Markers I can see:")

def Pick_up():
        print("picking up cube")
        robot.scissor_down(12)
        robot.pump = True
        robot.scissor_up(12)
        
def Drop():
        print("dropping cube")
        robot.scissor_up(12)
        robot.pump = False
        robot.scissor_down(12)

while True:
    if got_marker==False :
        time.sleep(1)
        markers = srRobot.camera.see()

        closest_marker = None
        min_dist = 9999999

        for marker in markers:
            if marker.id>=140 and marker.id<=159 and marker.position.distance < min_dist:
                closest_marker = marker
                min_dist = marker.position.distance

        if closest_marker == None:
            robot.right(30)
            time.sleep(2)
            continue


        #robot.DetectDistance(closest_marker)   
        print("Closest Marker:",closest_marker.id) 


        print("Turning",closest_marker.position.horizontal_angle)
        robot.right((closest_marker.position.horizontal_angle / math.pi)*180)

        print("Driving", (closest_marker.position.distance) / 1000)
        robot.forward((closest_marker.position.distance) / 1000)
        Pick_up()
        got_marker = True
    else:
        markers = srRobot.camera.see()

        found = None

        for marker in markers:
            if marker.id==197:
                found = marker

        if found == None:
            robot.right(30)
            time.sleep(2)
            continue


        #robot.DetectDistance(found)    

        print("Turning",found.position.horizontal_angle)
        robot.right((found.position.horizontal_angle / math.pi)*180)

        print("Driving", (found.position.distance) / 1000)
        robot.forward((found.position.distance) / 1000)
        Drop()
        got_marker = False

        robot.forward(-1)
        # move away from the plinth to see



'''robot.forward(1)


robot.right(90)'''

print("Loop ended.")
