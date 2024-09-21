#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0); // Open the default camera
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    cap >> frame; // Capture a frame
    if (frame.empty()) {
        std::cerr << "Error: Could not capture frame." << std::endl;
        return -1;
    }

    cv::Mat flippedFrame;
    cv::flip(frame, flippedFrame, 0); // Flip vertically

    cv::imshow("Original Frame", frame);
    cv::imshow("Flipped Frame", flippedFrame);
    cv::waitKey(0); // Wait for a key press

    cv::imwrite("flipped_image.jpg", flippedFrame); // Save flipped image

    cap.release(); // Release the camera
    cv::destroyAllWindows(); // Close all windows

    return 0;
}




