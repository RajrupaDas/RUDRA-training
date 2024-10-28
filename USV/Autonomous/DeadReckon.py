import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
import time
import serial
import smbus

gps_port = '/dev/ttyUSB0'
gps_baudrate = 9600

imu_address = 0x68
bus = smbus.SMBus(1)

ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
ekf.x = np.array([0, 0, 0, 0])
ekf.P *= 1000
ekf.R = np.diag([5, 5])
ekf.Q = np.eye(4) * 0.1

def transition_function(state, dt):
    x, vx, y, vy = state
    return np.array([x + vx * dt, vx, y + vy * dt, vy])

def transition_jacobian(state, dt):
    return np.array([[1, dt, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, dt],
                     [0, 0, 0, 1]])

def measurement_function(state):
    x, _, y, _ = state
    return np.array([x, y])

def measurement_jacobian(state):
    return np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])

def read_gps():
    with serial.Serial(gps_port, baudrate=gps_baudrate, timeout=1) as gps_data:
        line = gps_data.readline().decode('ascii', errors='replace')
        if line.startswith('$GPGGA'):
            parts = line.split(',')
            lat = float(parts[2])
            lon = float(parts[4])
            return lon, lat
    return 0, 0

def read_imu():
    def read_word(register):
        high = bus.read_byte_data(imu_address, register)
        low = bus.read_byte_data(imu_address, register + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value -= 65536
        return value

    ax = read_word(0x3B) / 16384.0
    ay = read_word(0x3D) / 16384.0
    return ax, ay

def dead_reckoning(ax, ay, dt, vx, vy, x, y):
    vx += ax * dt
    vy += ay * dt
    x += vx * dt + 0.5 * ax * dt ** 2
    y += vy * dt + 0.5 * ay * dt ** 2
    return x, y, vx, vy

vx, vy, x, y = 0, 0, 0, 0
last_time = time.time()

while True:
    gps_x, gps_y = read_gps()
    ax, ay = read_imu()
    
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    
    dr_x, dr_y, dr_vx, dr_vy = dead_reckoning(ax, ay, dt, vx, vy, x, y)
    
    ekf.F = transition_jacobian(ekf.x, dt)
    ekf.x = transition_function(ekf.x, dt)
    ekf.predict()

    z = np.array([gps_x, gps_y])
    ekf.H = measurement_jacobian(ekf.x)
    ekf.update(z, HJacobian=measurement_jacobian, Hx=measurement_function)
    
    x, vx, y, vy = ekf.x
    print(f"Estimated Position: x = {x:.6f}, y = {y:.6f}")
    
    vx, vy, x, y = dr_vx, dr_vy, dr_x, dr_y
    
    time.sleep(0.1)
