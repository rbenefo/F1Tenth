import numpy as np
import matplotlib.pyplot as plt



data = np.load("rangedata.npy")
# print(data[749])
minangle = -np.pi
maxangle = np.pi
anglevec = np.linspace(minangle, maxangle, data.shape[0])
plt.scatter(anglevec, data)
plt.xlabel("Angle (radians)")
plt.ylabel("Range (m)")
plt.show()