# This file is not used, and can be safely removed

import time
import matplotlib.pyplot as plt

currentError = 0
currentCount = 0
integral = 0
lastError = 0
SETPOINT = 9000

def updateCount(power, count, dt):
    return count + (power * dt * 500)

def pid(setpoint, current, Kp, Ki, Kd, dt):
    global integral, lastError
    error = setpoint - current
    P = Kp * error
    integral += error * dt
    integral = max(-1000, min(1000, integral))
    I = Ki * integral
    D = Kd * (error - lastError) / max(dt, 1e-6)
    lastError = error
    return P + I + D

x, y, setpoint = [], [], []

startTime = time.time()
lastTime = startTime
dt = 0.01

while time.time() - startTime < 10:
    currentTime = time.time()
    dt = currentTime - lastTime if currentTime > lastTime else dt
    lastTime = currentTime
    
    x.append(time.time() - startTime)
    y.append(currentCount)
    setpoint.append(SETPOINT)
    
    power = pid(SETPOINT, currentCount, 1000, 0.0001, 0.0001, currentTime - lastTime)
    power = max(-1, min(1, power))
    print(f"{power}, {currentCount}")
    currentCount = updateCount(power, currentCount, dt)
    
plt.plot(x, y, color="blue")
plt.plot(x, setpoint, color="orange")

plt.xlabel("Time (s)")
plt.ylabel("Count")
plt.legend()

plt.show()
    
