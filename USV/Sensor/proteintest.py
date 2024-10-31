import cv2
import numpy as np
import time

def detect_tube(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    blue_low, blue_high = np.array([90, 50, 50]), np.array([130, 255, 255])  #need to customise
    purple_low, purple_high = np.array([130, 50, 50]), np.array([160, 255, 255])  #this too

    mask_blue = cv2.inRange(hsv, blue_low, blue_high)
    mask_purple = cv2.inRange(hsv, purple_low, purple_high)
    mask_combined = cv2.bitwise_or(mask_blue, mask_purple)

    contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi = None
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) > 1000:  #can be adjusted for calibration
            x, y, w, h = cv2.boundingRect(largest_contour)
            roi = hsv[y:y+h, x:x+w]
            return roi
        else:
            print("No significant color area detected.")
            return None
    else:
        print("No color detected.")
        return None

def analyze_color(roi):
    if roi is not None:
        avg_color = cv2.mean(roi)[:3]

        if 90 <= avg_color[0] <= 130:
            print("Test result: Negative (Blue)")
        elif 130 < avg_color[0] <= 160:
            print("Test result: Positive (Purple)")
        else:
            print("No significant color change detected.")
    else:
        print("No ROI available.")

def capture_image(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("Error: Could not read frame.")
        return None

    return frame

print("Capturing initial image (transparent liquid)...")
initial_img = capture_image()

if initial_img is not None:
    cv2.imshow("Initial Test Tube", initial_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    time.sleep(5)  #time taken for test to complete

    print("Capturing second image (after color change)...")
    final_img = capture_image()

    if final_img is not None:
        cv2.imshow("Final Test Tube", final_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        print("Analyzing final image...")
        analyze_color(detect_tube(final_img))
