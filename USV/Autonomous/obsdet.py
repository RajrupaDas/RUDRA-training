import cv2
import numpy as np
from filterpy.kalman import KalmanFilter
import math

def run_obstacle_detection():
    calibration_data= np.load("stereo_calibration_data.npz")
    left_camera_matrix= calibration_data['left_camera_matrix']
    left_distortion= calibration_data['left_distortion']
    right_camera_matrix = calibration_data['right_camera_matrix']
    right_distortion= calibration_data['right_distortion']
    rotation_matrix= calibration_data['rotation_matrix']
    translation_vector= calibration_data['translation_vector']
    focal_length_value= calibration_data['focal_length']
    baseline_value= calibration_data['baseline']

    left_camera = cv2.VideoCapture(0)
    right_camera = cv2.VideoCapture(1)

    stereo_bm = cv2.StereoBM_create(numDisparities=16 * 5, blockSize=15)

    kalman_filter = KalmanFilter(dim_x=4, dim_z=2)
    kalman_filter.x = np.array([[0], [0], [0], [0]])
    kalman_filter.P *= 1000.
    kalman_filter.R = np.array([[10, 0], [0, 10]])
    kalman_filter.Q = np.eye(4)
    kalman_filter.F = np.array([[1, 0, 1, 0],
                                 [0, 1, 0, 1],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
    kalman_filter.H = np.array([[1, 0, 0, 0],
                                 [0, 1, 0, 0]])

    obstacle_detected = False
    detection_range_limit = 20.0

    while left_camera.isOpened() and right_camera.isOpened():
        ret_left, left_frame = left_camera.read()
        ret_right, right_frame = right_camera.read()

        if not ret_left or not ret_right:
            break

        gray_left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

        disparity_map = stereo_bm.compute(gray_left_frame, gray_right_frame).astype(np.float32) / 16.0
       
        filtered_disparity = cv2.medianBlur(disparity_map, 7) 

        depth_image = (focal_length_value * baseline_value) / (disparity_map + 1e-5)

        _, binary_threshold = cv2.threshold(gray_left_frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours_found, _ = cv2.findContours(binary_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_found:
            largest_contour = max(contours_found, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                x_position, y_position, width, height = cv2.boundingRect(largest_contour)
                center_x_position, center_y_position = x_position + width // 2, y_position + height // 2

                kalman_filter.predict()
                kalman_filter.update(np.array([[np.float32(center_x_position)], [np.float32(center_y_position)]]))

                predicted_state = kalman_filter.x
                predicted_x_position, predicted_y_position = int(predicted_state[0]), int(predicted_state[1])

                depth_value = depth_image[center_y_position, center_x_position]

                if depth_value <= detection_range_limit:
                    width_in_meters = (width * depth_value) / focal_length_value
                    offset_x_center = center_x_position - (gray_left_frame.shape[1] / 2)
                    angle_value = math.degrees(np.arctan(offset_x_center / focal_length_value))

                    cv2.rectangle(left_frame, (x_position, y_position), (x_position + width, y_position + height), (0, 255, 0), 2)
                    cv2.putText(left_frame, f'Depth: {depth_value:.2f} m', (x_position, y_position - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(left_frame, f'Width: {width_in_meters:.2f} m', (x_position, y_position - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(left_frame, f'Angle: {angle_value:.2f} degrees', (x_position, y_position - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    cv2.circle(left_frame, (predicted_x_position, predicted_y_position), 5, (255, 0, 0), -1)

                    if not obstacle_detected:
                        print(f"Detected obstacle - Depth: {depth_value:.2f} m, Width: {width_in_meters:.2f} m, Angle: {angle_value:.2f} degrees")
                        obstacle_detected = True
                else:
                    obstacle_detected = False
            else:
                obstacle_detected = False
        else:
            obstacle_detected = False

        cv2.imshow("Left Camera", left_frame)
        cv2.imshow("Right Camera", right_frame)
        normalized_disparity = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("Disparity", normalized_disparity.astype(np.uint8))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    left_camera.release()
    right_camera.release()
    cv2.destroyAllWindows()
