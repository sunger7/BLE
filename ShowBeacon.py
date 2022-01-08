import numpy as np
import matplotlib.pyplot as plt

beacon = []

with open("蓝牙信标数据.txt") as f:
    data = f.readlines()
    index = 0
    for i in range(len(data)):
        data[i].strip("\n")
        splits = data[i].split(',')
        #print(splits)
        if ( len(splits)<5 ):
            continue
        else:
            beacon.append(splits)
print(beacon)
Beacon_xy = np.zeros((len(splits),2))
for i in range(Beacon_xy.shape[0]):
    Beacon_xy[i,0] = splits[i][3]
    Beacon_xy[i,0] = splits[i][4]

plt.figure(figsize=(6,6))
plt.plot(Beacon_xy[0],Beacon_xy[1],'.')
plt.show()

