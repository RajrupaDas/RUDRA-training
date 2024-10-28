#This code is for testing and uses randomised data
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

time_data = []
concentration_data = []

def get_random_concentration():
    return np.random.uniform(0, 100)

fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlim(0, 10)
ax.set_ylim(0, 100)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Concentration (ppm)')
ax.set_title('Dynamic Gas Concentration Plot')

def update(frame):
    current_time = len(time_data)
    time_data.append(current_time)
    concentration = get_random_concentration()
    concentration_data.append(concentration)
    line.set_data(time_data, concentration_data)

    if current_time > 10:
        ax.set_xlim(current_time - 10, current_time)

    return line,

ani = FuncAnimation(fig, update, frames=np.arange(0, 200), blit=True, interval=1000)

plt.show()

