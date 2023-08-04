import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


file_list = ["qst3d_33", "qst3d_528", "qst3d_512"]
size = 32
enable_generate_gif = True

data_list = [np.loadtxt(f"{name}.data") for name in file_list]
X = [data[0] for data in data_list]
Y = [data[1] for data in data_list]
data_list = [data[2:] for data in data_list]


x_2d = [each.reshape((size,size)) for each in X]
y_2d = [each.reshape((size,size)) for each in Y]


fig = plt.figure(figsize=(12, 5))
ax = [fig.add_subplot(1, len(file_list), i + 1, projection="3d") for i in range(len(file_list))]


fig.show()

for i in range(data_list[0].shape[0]):
    # Plot a basic wireframe.
    Z = [each[i].reshape((size,size)) for each in data_list]
    
    # label x and y axis
    [ax[i].set_xlabel('X') for i in range(len(file_list))]
    [ax[i].set_ylabel('Y') for i in range(len(file_list))]
    # Update plot data and redraw
    [ax[i].set_zlim(0, 0.05) for i in range(len(file_list))]
    [ax[i].plot_wireframe(x_2d[i], y_2d[i], Z[i], rstride=1, cstride=1) for i in range(len(file_list))]
    
    if enable_generate_gif:
        plt.savefig(f"./img/test_{i}.png")
    # Wait for a short time before updating again
    plt.pause(0.02)
    [ax[i].clear() for i in range(len(file_list))]


if enable_generate_gif:
    images = [Image.open(f"./img/test_{n}.png") for n in range(data_list[0].shape[0])]
    images[0].save("test.gif", save_all=True, append_images=images[1:], duration=80, loop=0)

