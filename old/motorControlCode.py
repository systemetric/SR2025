COUNTS_PER_REVOL = 2720
revolDistance = 0.392 #m
countAccuracy = 30
    
def clamp(val, min, max):
    if (val < min):
        return min
    elif (val > max):
        return max
    else:
        return val

def powerByCount(pCount, pidObject):
    return pidObject(pCount) 
    
P, I, D = [1, 0.01, 0.5]
def RobotDrive(pDistance, pMode = 2):

    #Calculate target
    targetCount = int(COUNTS_PER_REVOL * pDistance / revolDistance)

    #Initialise PID objects
    m0PID = PID(P, I, D, setpoint = targetCount)
    m0PID.output_limits = (-1,1)
    m1PID = PID(P, I, D, setpoint = targetCount)
    m1PID.output_limits = (-1,1)
    
    #reset motor counts
    robot.arduino.command("c")
    initialTime = time.time()

    reached = False

    """
    debug(f"[{time.time() - initialTime}]")
    debug(f"Received Data: '{receivedData}'")
    debug(f"Bad message received \"{receivedData}\"")
    """
        
    while not reached:

        receivedData = robot.arduino.command("m")
        if receivedData[-1] != '|':
            
            continue
        
        m0Count, m1Count = list(map(int, receivedData[:-1].split(";")))
        if (pMode == 0 or pMode == 2):
            mb.motors[0].power = powerByCount(m0Count, m0PID)
            reached = abs(targetCount - m0Count) < countAccuracy

        if (pMode == 1 or pMode == 2):
            mb.motors[1].power = powerByCount(m1Count, m1PID)
            reached = abs(targetCount - m1Count) < countAccuracy

        

def RobotRotate(pAngle, pMode):
    arcRadius = 0.425 #m
    halfArc = arcRadius * math.pi #m

    RobotDrive(halfArc * (pAngle / 180), pMode)
