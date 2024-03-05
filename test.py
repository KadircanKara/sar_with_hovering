import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
from tempfile import TemporaryFile

a = 1
b = 2

a,b=b,a

print(a,b)

'''# Sample data for two drones
x_matrix = np.load('x_matrix.npy', allow_pickle=True)
y_matrix = np.load('y_matrix.npy', allow_pickle=True)


fig, axis = plt.subplots()

axis = plt.axes(xlim=(-50, 450), ylim=(-50, 450))
axis.set_xticks(range(0, 451, 50))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
axis.set_yticks(range(0, 451, 50))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
plt.grid(linestyle='--')

# np.random.rand(3,)
drone_colors = ['k','g','r','c','m','y','b']
scatter_plots = [axis.scatter([], [], color=drone_colors[i], label=f'Drone {i}') for i in range(5)]
lines = [plt.plot([], color=drone_colors[i], label=f"Drone {i}") for i in range(5)]
# lines = [axis.line([], [], color=drone_colors[i], label=f'Drone {i}') for i in range(5)]

def animate(frame):

    for i in range(5):
        x_scatter = x_matrix[i, frame+1]
        y_scatter = y_matrix[i, frame+1]
        scatter_plots[i].set_offsets(np.column_stack([x_scatter, y_scatter]))
        x_line = x_matrix[i, :frame+1]
        y_line = y_matrix[i, :frame+1]
        # print("-->",lines[i])
        lines[i][0].set_data((x_line, y_line))

anim = FuncAnimation(fig, animate, frames=x_matrix.shape[1], interval=100, blit=False)
plt.show()'''