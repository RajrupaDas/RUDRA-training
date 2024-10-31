import csv
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ser = serial.Serial('/dev/ttyUSB0', 9600)

time_data = []
concentration_data = []
time_step = 0

with open("methane_concentration_data.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Methane Concentration (arbitrary units)"])

    plt.style.use('seaborn-darkgrid')
    fig, ax = plt.subplots()
    ax.set_title("Methane Concentration vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Methane Concentration (arbitrary units)")

    def update(frame):
        global time_step
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            try:
                concentration = float(line)
                time_data.append(time_step)
                concentration_data.append(concentration)
                
                writer.writerow([time_step, concentration])
                
                time_step += 0.5
                ax.clear()
                ax.plot(time_data, concentration_data, color='blue')
                ax.set_title("Methane Concentration vs Time")
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Methane Concentration (arbitrary units)")
            except ValueError:
                pass

    ani = FuncAnimation(fig, update, interval=500)
    plt.show()
  
