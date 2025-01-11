from datetime import datetime
        
class MyRobotDebug:
    DEBUG_FILE = None
    DEBUG = True
    DEBUG_PASS_THROUGH = False

    def __init__(self, enable = True, passThrough = False):
        self.DEBUG = enable
        self.DEBUG_PASS_THROUGH = passThrough
        thisTime = datetime.now()
        thisTimeFmt = thisTime.strftime("%d.%m.%Y_%H.%M.%S")
        self.DEBUG_FILE = open(f"log_{thisTimeFmt}.txt", "w")

    def __del__(self):
        self.stop()

    def setEnabled(self, enabled):
        self.DEBUG = enabled

    def setPassThrough(self, passThrough):
        self.DEBUG_PASS_THROUGH = passThrough

    def stop(self):
        self.DEBUG_FILE.close()

    def debug(self, *args, end="\n"):
        if self.DEBUG:
            for s in args:
                self.DEBUG_FILE.write(s)
                if self.DEBUG_PASS_THROUGH:
                    print(s, end="")
            self.DEBUG_FILE.write(end)
            self.DEBUG_FILE.flush()
            if self.DEBUG_PASS_THROUGH:
                print(end, end="")