from datetime import datetime
        
class MyRobotDebug:
    __DEBUG_FILE = None
    __DEBUG = True
    __DEBUG_PASS_THROUGH = False

    def __init__(self, enable = True, passThrough = False):
        self.__DEBUG = enable
        self.__DEBUG_PASS_THROUGH = passThrough
        thisTime = datetime.now()
        thisTimeFmt = thisTime.strftime("%d.%m.%Y_%H.%M.%S")
        self.__DEBUG_FILE = open(f"log_{thisTimeFmt}.txt", "w")

    def __del__(self):
        self.stop()

    def setEnabled(self, enabled):
        self.__DEBUG = enabled

    def setPassThrough(self, passThrough):
        self.__DEBUG_PASS_THROUGH = passThrough

    def stop(self):
        self.__DEBUG_FILE.close()

    def debug(self, *args, end="\n"):
        if self.__DEBUG:
            for s in args:
                self.__DEBUG_FILE.write(str(s))
                if self.__DEBUG_PASS_THROUGH:
                    print(str(s), end="")
            self.__DEBUG_FILE.write(end)
            if self.__DEBUG_PASS_THROUGH:
                print(end, end="")