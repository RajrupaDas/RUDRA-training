import cv2
import numpy as np
import open3d as o3d

calibration_data = np.load('stereo_calibration_data.npz')
camera_matrix_left = calibration_data['camera_matrix_left']
camera_matrix_right = calibration_data['camera_matrix_right']
distortion_coeffs_left = calibration_data['distortion_coeffs_left']
distortion_coeffs_right = calibration_data['distortion_coeffs_right']
R = calibration_data['R']
T = calibration_data['T']

stereo_matcher = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)

left_camera = cv2.VideoCapture(0)
right_camera = cv2.VideoCapture(1)

while True:
    ret_left, frame_left = left_camera.read()
    ret_right, frame_right = right_camera.read()

    if not ret_left or not ret_right:
        print("Failed to grab frames")
        break

    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    disparity = stereo_matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0

    h, w = gray_left.shape
    Q = np.array([[1, 0, 0, -0.5*w],
                   [0, -1, 0, 0.5*h], 
                   [0, 0, 0, -camera_matrix_left[0, 0]], 
                   [0, 0, 1/T[0, 0], 0]])
    
    points = cv2.reprojectImageTo3D(disparity, Q)
    mask = disparity > disparity.min()
    out_points = points[mask]

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(out_points)

    colors = np.zeros((out_points.shape[0], 3))
    colors[:, 0] = frame_left[mask][:, 0] / 255.0
    colors[:, 1] = frame_left[mask][:, 1] / 255.0
    colors[:, 2] = frame_left[mask][:, 2] / 255.0
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([point_cloud])

    cv2.imshow("Left Camera", frame_left)
    cv2.imshow("Right Camera", frame_right)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

left_camera.release()
right_camera.release()
cv2.destroyAllWindows()
