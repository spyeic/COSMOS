# imports important packages similar to "#include" in C
import numpy as np #this has useful data structures like arrays which will be discussed more later
from matplotlib import pyplot as plt #this is for plotting nicely
import sys, os

if len(sys.argv) < 2:
    print("Usage: python graph.py <filename>...")
    exit(1)

graph_paths = sys.argv[1:]
print("Graphing file(s): ", graph_paths)

# This will import the data we exported from C
# in the place of "trajectory.txt" you should use your the name you gave your text file containing
# the "data" from your simulation, which can be the full path if you do not want to store the data 
# in the same directory as this program.
graph_datas = [np.loadtxt(each) for each in graph_paths]

# since we don't know how this was imported, I will just check by asking the program to tell me the 
# dimensions of trajectory
# print ("number of dimensions: ",trajectory.ndim)
# print ("shape: ", trajectory.shape)


# Now we know how the array "trajectory" is shaped, so we can plot the trajectory!
# If the axes come out inverted, you can always switch the contents in the plot function

plt.xlabel("t (s)");
plt.ylabel("x (m)");

colors = ["red", "blue", "green", "orange", "purple", "black", "yellow", "pink", "brown", "gray"]

for i, each in enumerate(graph_datas):
    plt.plot(each[:,0], each[:,1], color=colors[i % len(colors)], label=graph_paths[i])

plt.legend(loc='upper right')

plt.show()
