# imports important packages similar to "#include" in C
import numpy as np #this has useful data structures like arrays which will be discussed more later
from matplotlib import pyplot as plt #this is for plotting nicely
import subprocess

subprocess.run("./a.out")

# This will import the data we exported from C
# in the place of "trajectory.txt" you should use your the name you gave your text file containing
# the "data" from your simulation, which can be the full path if you do not want to store the data 
# in the same directory as this program.
trajectory = np.loadtxt("kepler.data")

# since we don't know how this was imported, I will just check by asking the program to tell me the 
# dimensions of trajectory
print ("number of dimensions: ",trajectory.ndim)
print ("shape: ", trajectory.shape)


# Now we know how the array "trajectory" is shaped, so we can plot the trajectory!
# If the axes come out inverted, you can always switch the contents in the plot function

plt.plot(trajectory[:,0], trajectory[:,1], color = 'purple', label = "Earth orbit")
plt.xlabel("x (m)");
plt.ylabel("y (m)");
plt.legend(loc='lower left');

plt.show()