import os,sys,math,time
from myrobot import *

class MyRobotCamera:
    __MARKER_PALLET = 0
    __MARKER_PLINTH = 1

    def __init__(self, robot, min_dist=999999):
        self.MIN_DIST = min_dist
        self.ROBOT = robot
        self.__NEXT_OBJ = self.__MARKER_PALLET

    def find_pallet_markers(self):
        # markers = self.ROBOT.ROBOT.camera.see(save=f"capture.jpg")
        markers = self.ROBOT.ROBOT.camera.see()

        palletMarkers = []
        for marker in markers:
            if marker.id >= 100 and marker.id <= 179 and marker.position.distance < self.MIN_DIST:
                palletMarkers.append(marker)

        # sorts palletMarkers into least to highest distance
        for i in range(len(palletMarkers) - 1):
            if palletMarkers[i].position.distance > palletMarkers[i + 1].position.distance:
                palletMarkers[i], palletMarkers[i + 1] = palletMarkers[i + 1], palletMarkers[i]
        
        for palletMarker in palletMarkers:
            self.show_marker(palletMarker)

        return palletMarkers

    def find_high_rise_markers(self):
        # markers = self.ROBOT.ROBOT.camera.see(save=f"capture.jpg")
        markers = self.ROBOT.ROBOT.camera.see()

        highRiseMarkers = []
        for marker in markers:
            if marker.id >= 195 and marker.id <= 198 and marker.position.distance < self.MIN_DIST:
                highRiseMarkers.append(marker)

        # sorts highRiseMarkers into least to highest distance
        for i in range(len(highRiseMarkers) - 1):
            if highRiseMarkers[i].position.distance > highRiseMarkers[i + 1].position.distance:
                highRiseMarkers[i], highRiseMarkers[i + 1] = highRiseMarkers[i + 1], highRiseMarkers[i]
        
        return highRiseMarkers
     
    def show_marker(self, mx):
        print(f"ID = {mx.id}    Distance = {mx.position.distance}")
        self.ROBOT.DEBUGGER.debug(f"ID = {mx.id}    Distance = {mx.position.distance}")

    def go_to_cube(self, m):
        for i in range(4):
            self.ROBOT.right(m.position.horizontal_angle, isRadians=True)
            self.ROBOT.forward(self.dist_func(m.position.distance) / (4 - i))



'''
def CameraMain():
    robot = MyRobot(dbgEnabled=False)
    got_marker = False

    mrc = MyRobotCamera(robot)

    n = 0

    while True:
        if got_marker==False :
            time.sleep(1)

            markers = mrc.find_pallet_markers();

            if len(markers) < 1:
                robot.right(10)
                continue;

            closest_marker = markers[0]

            print("Closest Marker:",closest_marker.id)

            print("Turning",closest_marker.position.horizontal_angle)
            robot.right(closest_marker.position.horizontal_angle, isRadians=True)

            print("Driving", (closest_marker.position.distance) / 1000)
            robot.forward((closest_marker.position.distance) / 1000)
            robot.grab()
            got_marker = True
        else:

            markers = mrc.find_high_rise_markers();

            if len(markers) < 1:
                robot.right(10);
                continue;
            
            found = markers[0]; 

            print("Turning",found.position.horizontal_angle)
            robot.right(found.position.horizontal_angle, isRadians=True)

            print("Driving", (found.position.distance) / 1000)
            robot.forward((found.position.distance) / 1000)
            robot.drop()
            got_marker = False

            robot.reverse(1)
'''