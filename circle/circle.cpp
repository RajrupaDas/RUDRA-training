#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Load the image from the Downloads folder
    std::string imagePath = "/home/rajrupa-das/Downloads/your_image.jpg";
    cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);

    if (img.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

    // Detect circles using HoughCircles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/8, 200, 100, 0, 0);

    // Draw the detected circles
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3f c = circles[i];
        cv::Point center(cvRound(c[0]), cvRound(c[1]));
        int radius = cvRound(c[2]);
        // Draw the circle center
        cv::circle(img, center, 3, cv::Scalar(0, 255, 0), -1);
        // Draw the circle outline
        cv::circle(img, center, radius, cv::Scalar(0, 0, 255), 3);
    }

    // Show the result
    cv::imshow("Detected Circles", img);
    cv::waitKey(0);

    return 0;
}
