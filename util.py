import os, re

class MyRobotDebug:
    __DEBUG_FILE = None
    __DEBUG = True
    __DEBUG_PASS_THROUGH = False

    def __init__(self, enable = True, passThrough = False):
        self.__DEBUG = enable
        self.__DEBUG_PASS_THROUGH = passThrough
        if enable:
            self.__DEBUG_FILE = open(self.__getNewLogFileName(), "w")

    def __del__(self):
        self.stop()

    def __getNewLogFileName(self):
        # Get log file name based on last. 'logXX.txt'
        logFiles = [f for f in os.listdir('.') if (f.endswith('.txt') and f.startswith('log'))]
        logFiles.sort()

        logFiles.remove("log.txt")

        n = 0
        if len(logFiles) > 0:
            n = int(list(map(int, re.findall("\d+",logFiles[len(logFiles) - 1])))[0]) + 1

        return f"log{n}.txt"
    

    def setEnabled(self, enabled):
        self.__DEBUG = enabled
        if self.__DEBUG_FILE == None:
            self.__DEBUG_FILE = open(self.__getNewLogFileName(), "w")

    def setPassThrough(self, passThrough):
        self.__DEBUG_PASS_THROUGH = passThrough

    def stop(self):
        # Flush and close log file
        self.__DEBUG_FILE.flush()
        self.__DEBUG_FILE.close()

    def debug(self, *args, end="\n"):
        if self.__DEBUG:
            for s in args:
                # Print each argument
                self.__DEBUG_FILE.write(str(s))
                if self.__DEBUG_PASS_THROUGH:
                    print(str(s), end="")
            self.__DEBUG_FILE.write(end)
            if self.__DEBUG_PASS_THROUGH:
                print(end, end="")