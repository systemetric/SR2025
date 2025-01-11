import matplotlib.pyplot as plt
import re

lines = [line.strip().split() for line in open("log.txt","r").readlines()]

print(lines[0])

X = []
Y = []
 
i = 3

timeList = [] 

for a,_,_,_,_,b in lines[1:]:
    Y.append(int(b))
    A,B,C,D = re.findall("(\d+)",a[1:-1])
    X.append((time:=int(A) * 3600 + int(B) * 60 + int(C) + float(f"0.{D}"))) 
    timeList.append(time)

print(sorted([timeList[i+1] - timeList[i] for i in range(len(timeList)-1)])[::-1])

plt.plot(X,Y, marker="x", markersize=1, linestyle=None)
plt.xlabel("Time")
plt.ylabel("Count")
plt.show()