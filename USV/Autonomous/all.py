import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
import math
import time
import matplotlib.pyplot as plt


s_lat, s_lon = 34.0522, -118.2437  
e_lat, e_lon = 34.0525, -118.2430  

n_steps = 20 
lat_step = (e_lat - s_lat) / n_steps
lon_step = (e_lon - s_lon) / n_steps

# EKF Initialization
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
ekf.x = np.array([s_lon, 0, s_lat, 0])
ekf.P *= 1000
ekf.R = np.diag([0.0001, 0.0001])
ekf.Q = np.eye(4) * 0.1

est_pos = []

# Dead Reckoning 
def f(state, dt):
    x, vx, y, vy = state
    return np.array([x + vx * dt, vx, y + vy * dt, vy])

def Jf(state, dt):
    return np.array([[1, dt, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, dt],
                     [0, 0, 0, 1]])

def h(state):
    x, _, y, _ = state
    return np.array([x, y])

def Jh(state):
    return np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])

def sim_gps(step):
    return (s_lon + lon_step * step, s_lat + lat_step * step)

def sim_mpu():
    return 0.0, 0.0

def dr(ax, ay, dt, vx, vy, x, y):
    vx += ax * dt
    vy += ay * dt
    x += vx * dt + 0.5 * ax * dt ** 2
    y += vy * dt + 0.5 * ay * dt ** 2
    return x, y, vx, vy

# Obstacle Edge Local Coordinates 
def obstacle_edges_camera(depth, angle, width):
    angle_rad = math.radians(angle)
    center_x = depth * math.cos(angle_rad)
    center_y = depth * math.sin(angle_rad)
    half_width_offset = width / 2

    left_x = center_x + half_width_offset * math.sin(angle_rad)
    left_y = center_y - half_width_offset * math.cos(angle_rad)
    right_x = center_x - half_width_offset * math.sin(angle_rad)
    right_y = center_y + half_width_offset * math.cos(angle_rad)

    return {
        "center": (center_x, center_y),
        "left_edge": (left_x, left_y),
        "right_edge": (right_x, right_y)
    }

# Global Coordinates based on Dead Reckoning Position
def global_coordinates(local_coords, ekf_lat, ekf_lon):
    lat_conversion = 0.00001  # Scale factor for latitude
    lon_conversion = 0.00001  # Scale factor for longitude
    global_coords = (
        ekf_lat + local_coords[1] * lat_conversion,
        ekf_lon + local_coords[0] * lon_conversion
    )
    return global_coords

# Sim loop for Dead Reckoning and Obstacle Detection
vx, vy, x, y = 0, 0, s_lon, s_lat
last_time = time.time()

for step in range(n_steps):
    gps_x, gps_y = sim_gps(step)
    ax, ay = sim_mpu()
    
    curr_time = time.time()
    dt = curr_time - last_time
    last_time = curr_time
    
    # Update Dead Reckoning using EKF
    dr_x, dr_y, dr_vx, dr_vy = dr(ax, ay, dt, vx, vy, x, y)
    ekf.F = Jf(ekf.x, dt)
    ekf.x = f(ekf.x, dt)
    ekf.predict()

    z = np.array([gps_x, gps_y])
    ekf.H = Jh(ekf.x)
    ekf.update(z, HJacobian=Jh, Hx=h)
    
    x, vx, y, vy = ekf.x
    est_pos.append((y, x))
    print(f"Estimated Position: Latitude = {y:.6f}, Longitude = {x:.6f}")

    
    obstacle_depth = 10  # meters
    obstacle_angle = 15  # degrees
    obstacle_width = 1   # meters

    # edges relative to camera
    edge_coordinates = obstacle_edges_camera(obstacle_depth, obstacle_angle, obstacle_width)

    # global coordinates using dead reckoning position
    center_global = global_coordinates(edge_coordinates["center"], y, x)
    left_edge_global = global_coordinates(edge_coordinates["left_edge"], y, x)
    right_edge_global = global_coordinates(edge_coordinates["right_edge"], y, x)

    print(f"Obstacle {step+1} - Center Global Coordinates: {center_global}")
    print(f"Obstacle {step+1} - Left Edge Global Coordinates: {left_edge_global}")
    print(f"Obstacle {step+1} - Right Edge Global Coordinates: {right_edge_global}")
    
    #update dead reckoning values for next iteretion
    vx, vy, x, y = dr_vx, dr_vy, dr_x, dr_y
    
    time.sleep(0.1)

print("Goal reached!")

#plotting
lats, lons = zip(*est_pos)
plt.plot(lons, lats, marker='o', label='Estimated Path')
plt.plot(s_lon, s_lat, 'go', label='Start')
plt.plot(e_lon, e_lat, 'ro', label='End')
plt.plot([s_lon, e_lon], [s_lat, e_lat], 'b--', label='Straight Line')
plt.title('Path from Start to Goal')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.grid()
plt.show()
