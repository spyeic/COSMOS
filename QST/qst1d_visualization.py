import matplotlib.pyplot as plt
import numpy as np


plt.ion()
data = np.loadtxt("qstai.txt") # bing chilling
print(data.shape)


for i in range(data.shape[0]):
    plt.plot(np.arange(32), data[i])
    plt.draw()
    plt.pause(0.001)
    plt.clf()