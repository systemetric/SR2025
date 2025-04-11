import os,sys,math,time
from myrobot import *

class MyRobotCamera:
    __MARKER_PALLET = 0
    __MARKER_PLINTH = 1

    def __init__(self, robot, zone, min_dist=999999):
        self.MIN_DIST = min_dist
        self.ROBOT = robot
        self.__NEXT_OBJ = self.__MARKER_PALLET
        self.zone = zone

    def find_pallet_markers(self, ignored_markers:list = []):
        # markers = self.ROBOT.ROBOT.camera.see(save=f"capture.jpg")
        markers = self.ROBOT.camera.see()

        MIN_PALLET = (self.zone * 20) + 100
        MAX_PALLET = MIN_PALLET + 19

        print("I'm looking for pallets from", MIN_PALLET, "to", MAX_PALLET)

        palletMarkers = []
        for marker in markers:
            if marker.id >= 100 and marker.id <= 179 and marker.position.distance < self.MIN_DIST and marker.id not in ignored_markers:
                palletMarkers.append(marker)

        # sorts palletMarkers into least to highest distance
        palletMarkers.sort(key=lambda m: m.position.distance)

        # for i in range(len(palletMarkers) - 1):
        #     if palletMarkers[i].position.distance > palletMarkers[i + 1].position.distance:
        #         palletMarkers[i], palletMarkers[i + 1] = palletMarkers[i + 1], palletMarkers[i]
        
        for palletMarker in palletMarkers:
            self.show_marker(palletMarker)

        return palletMarkers

    def find_high_rise_markers(self):
        # markers = self.ROBOT.ROBOT.camera.see(save=f"capture.jpg")
        markers = self.ROBOT.camera.see()

        highRiseMarkers = []
        for marker in markers:
            if marker.id >= 195 and marker.id <= 198 and marker.position.distance < self.MIN_DIST:
                highRiseMarkers.append(marker)

        # sorts highRiseMarkers into least to highest distance
        for i in range(len(highRiseMarkers) - 1):
            if highRiseMarkers[i].position.distance > highRiseMarkers[i + 1].position.distance:
                highRiseMarkers[i], highRiseMarkers[i + 1] = highRiseMarkers[i + 1], highRiseMarkers[i]
        
        return highRiseMarkers

    def is_pallet_on_plinth(self, markers: list, plinth_id: int) -> bool:
        plinth = None
        
        for marker in markers:
            if marker.id == plinth_id:
                plinth = marker
                break
        
        if plinth == None:
            return False
        
        for marker in markers:
            plinth_scaled_angle = plinth.position.horizontal_angle / plinth.position.distance
            marker_scaled_angle = marker.position.horizontal_angle / marker.position.distance
            if marker.id != plinth_id and marker_scaled_angle >= plinth_scaled_angle - 0.1 and marker_scaled_angle <= plinth_scaled_angle + 0.1 and marker.position.vertical_angle > plinth.position.vertical_angle:
                return True
        
        return False

    def find_all_markers(self):
        markers = self.ROBOT.camera.see()

        # sorts markers into least to highest distance
        for i in range(len(markers) - 1):
            if markers[i].position.distance > markers[i + 1].position.distance:
                markers[i], markers[i + 1] = markers[i + 1], markers[i]
        
        return markers

    def show_marker(self, mx):
        print(f"ID = {mx.id}    Distance = {mx.position.distance}")
        # self.ROBOT.DEBUGGER.debug(f"ID = {mx.id}    Distance = {mx.position.distance}")