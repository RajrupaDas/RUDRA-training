import cv2
import numpy as np
from filterpy.kalman import KalmanFilter
import math

def run_obstacle_detection():
    
    focal_length_value = 1.0  #
    baseline_value = 0.1  
    left_camera_matrix = np.array([[focal_length_value, 0, 0],
                                    [0, focal_length_value, 0],
                                    [0, 0, 1]], dtype=np.float32)

    left_distortion = np.zeros(5) 
    right_camera_matrix = left_camera_matrix  
    right_distortion = np.zeros(5) 
    rotation_matrix = np.eye(3)  
    translation_vector = np.array([[baseline_value], [0], [0]]) 

    
    left_frame = cv2.imread('scene1.row3.col3.ppm')
    right_frame = cv2.imread('scene1.row3.col4.ppm')

    
    if left_frame is None or right_frame is None:
        print("Error loading images.")
        return

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

    detection_range_limit = 20.0
    object_counter = 0 

    
    gray_left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
    gray_right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

    
    disparity_map = stereo_bm.compute(gray_left_frame, gray_right_frame).astype(np.float32) / 16.0

    
    normalised_disparity = cv2.normalize(disparity_map, None, 0, 225, cv2.NORM_MINMAX)
    disparity_map_8u = np.uint8(normalised_disparity) 
    filtered_disparity = cv2.medianBlur(disparity_map_8u, 7)

    
    unique_values = np.unique(filtered_disparity)
    print("Unique values in filtered disparity:", unique_values)

    min_value = np.min(filtered_disparity)
    max_value = np.max(filtered_disparity)
    mean_value = np.mean(filtered_disparity)
    print(f"Filtered Disparity - Min: {min_value}, Max: {max_value}, Mean: {mean_value}")


    depth_image = np.divide(focal_length_value * baseline_value, 
                             filtered_disparity.astype(np.float32), 
                             out=np.zeros_like(filtered_disparity, dtype=np.float32), 
                             where=filtered_disparity != 0)
    

    print("Depth Image - Min:", np.min(depth_image), "Max:", np.max(depth_image))

   
    disparity_pgm = np.clip(filtered_disparity * 16, 0, 65535).astype(np.uint16) #uint16 for .pmg
    cv2.imwrite('disparity_output.pgm', disparity_pgm)

    
    _, binary_threshold = cv2.threshold(gray_left_frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours_found, _ = cv2.findContours(binary_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_found:
        for contour in contours_found:
            if cv2.contourArea(contour) > 100:
                object_counter += 1 
                x_position, y_position, width, height = cv2.boundingRect(contour)
                center_x_position, center_y_position = x_position + width // 2, y_position + height // 2

                kalman_filter.predict()
                kalman_filter.update(np.array([[np.float32(center_x_position)], [np.float32(center_y_position)]]))

                predicted_state = kalman_filter.x
                predicted_x_position, predicted_y_position = int(predicted_state[0].item()), int(predicted_state[1].item())

                depth_value = depth_image[center_y_position, center_x_position]

                if depth_value <= detection_range_limit:
                    width_in_meters = (width * depth_value) / focal_length_value
                    
                    
                    depth_value_scaled = depth_value *100  
                    width_in_meters_scaled = width_in_meters *2 
                    
                    offset_x_center = center_x_position - (gray_left_frame.shape[1] / 2)
                    angle_value = math.degrees(np.arctan(offset_x_center / focal_length_value))

                    
                    cv2.rectangle(left_frame, (x_position, y_position), (x_position + width, y_position + height), (0, 255, 0), 2)
                    cv2.circle(left_frame, (center_x_position, center_y_position), 5, (255, 0, 0), -1)
                    cv2.putText(left_frame, f'Obj: {object_counter}', (x_position, y_position - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                
                    print(f"Detected obstacle {object_counter} - Depth: {depth_value_scaled:.2f} m, Width: {width_in_meters_scaled:.2f} m, Angle: {angle_value:.2f} degrees")

    
    cv2.imshow("Left Camera", left_frame)
    normalized_disparity = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX)
    cv2.imshow("Disparity", normalized_disparity.astype(np.uint8))

    cv2.waitKey(0)  
    cv2.destroyAllWindows()

run_obstacle_detection()
