import cv1
import numpy as np

def load_images():
    img1_path = '/home/quantumvortex/captured_photo.jpg'  # Ensure this path is correct
    img2_path = '/home/quantumvortex/1captured_photo.jpg'  # Ensure this path is correct
    
    img1 = cv2.imread(img1_path)
    img2 = cv2.imread(img2_path)

    print(f"Loaded Image 1: {img1_path} - {img1 is not None}")
    print(f"Loaded Image 2: {img2_path} - {img2 is not None}")

    if img1 is None or img2 is None:
        print("Error loading images!")
        exit()

    return img1, img2

def detect_features(img1, img2):
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create()
    keypoints1, descriptors1 = orb.detectAndCompute(gray1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(gray2, None)

    print(f"Keypoints in Image 1: {len(keypoints1)}, Keypoints in Image 2: {len(keypoints2)}")
    
    return keypoints1, descriptors1, keypoints2, descriptors2

def match_features(descriptors1, descriptors2):
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)

    print(f"Number of matches found: {len(matches)}")
    return matches

def calculate_distance(matches, baseline, focal_length):
    disparities = [match.distance for match in matches]
    print(f"Disparities: {disparities}")  # Debug disparities
    average_disparity = np.mean(disparities) if disparities else 0
    print(f"Average Disparity: {average_disparity}")

    if average_disparity == 0:
        print("Disparity is zero; can't calculate distance.")
        return None

    # Calculate distance using the formula: Distance = (Focal Length * Baseline) / Disparity
    distance = (focal_length * baseline) / average_disparity
    print(f"Calculated Distance: {distance}")  # Print distance
    return distance

def detect_largest_contour(image_path):
    # Load the image
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to the image
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Threshold the image to get a binary image
    _, thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours in the image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the largest contour
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)  # Get bounding box of the largest contour
        return (w, (x, y, w, h))  # Return width and bounding box
    else:
        return None

def calculate_distance_with_contour(focal_length, width_of_object, actual_width):
    # Distance calculation formula
    distance = (focal_length * actual_width) / width_of_object
    return distance

def main():
    # Part 1: Feature matching and distance calculation
    img1, img2 = load_images()
    keypoints1, descriptors1, keypoints2, descriptors2 = detect_features(img1, img2)
    matches = match_features(descriptors1, descriptors2)

    img_matches = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imshow('Matches', img_matches)
    cv2.waitKey(0)

    baseline = 0.0254  # Distance moved in meters (1 inch)
    focal_length = float(input("Enter the focal length in pixels: "))  # Accept focal length

    distance = calculate_distance(matches, baseline, focal_length)

    if distance is not None:
        print(f"Estimated Distance from Camera to Object (Feature Matching): {distance:.2f} meters")
    
    # Part 2: Contour detection and distance calculation
    image_path = '/home/quantumvortex/captured_photo.jpg'  # Replace with your image file path
    result = detect_largest_contour(image_path)

    if result:
        width_of_object = result[0]  # Width of the object from the image

        # Accept user input for focal length and actual width of the object
        actual_width = float(input("Enter the actual width of the object in meters: "))  # Actual width of the object in meters

        # Calculate distance
        distance_contour = calculate_distance_with_contour(focal_length, width_of_object, actual_width)

        print(f"Estimated Distance from Camera to Object (Contour Detection): {distance_contour:.2f} meters")
    else:
        print("No object detected in the image.")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

