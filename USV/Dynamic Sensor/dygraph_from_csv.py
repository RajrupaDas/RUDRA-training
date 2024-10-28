import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

time_data = []
mq2_data = []
mq3_data = []
mq4_data = []

fig, ax = plt.subplots()
line1, = ax.plot([], [], label='MQ-2', color='blue')
line2, = ax.plot([], [], label='MQ-3', color='orange')
line3, = ax.plot([], [], label='MQ-4', color='green')

ax.set_xlim(0, 60)
ax.set_ylim(0, 1000)
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Gas Concentration (ppm)')
ax.legend(loc='upper right')
ax.grid()

def update_data():
    try:
        with open('gas_data.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                current_time = float(row[0])
                mq2_concentration = float(row[1])
                mq3_concentration = float(row[2])
                mq4_concentration = float(row[3])
                time_data.append(current_time)
                mq2_data.append(mq2_concentration)
                mq3_data.append(mq3_concentration)
                mq4_data.append(mq4_concentration)
    except FileNotFoundError:
        print("File not found. Make sure the CSV file exists.")
    except Exception as e:
        print(f"Error reading data: {e}")

def animate(i):
    update_data()
    if time_data:
        line1.set_data(time_data, mq2_data)
        line2.set_data(time_data, mq3_data)
        line3.set_data(time_data, mq4_data)
        ax.set_xlim(0, max(time_data) + 5)
        ax.relim()
        ax.autoscale_view()
    plt.draw()

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()
