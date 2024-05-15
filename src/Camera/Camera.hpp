#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/inotify.h>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>
#include <algorithm>

using namespace cv;


class Camera {
    private:
    /* Object detection */
    int low_H = 87, low_S = 130, low_V = 43;
    int high_H = 113, high_S = 255, high_V = 255;
    const int max_value_H = 360 / 2;
    const int max_value = 255;
    cv::Size kernel = cv::Size(5, 5);
    int dilate_size = 15;
    int max_dilate_size = 40;
    cv::VideoCapture cap;

    /* Config */
    int inotifyFd;
    int watchFd;
    std::string filePath = "./config/thresholds.conf";

    std::thread objThread;
    public:
    Camera() {
        cap.open(0);
        if (!cap.isOpened()) {
            std::cerr << "Error: Couldn't open the capture device." << std::endl;
            exit(1);
        }
        CameraConfig Config;
        Config.loadConfig();
    }

    void start() {
        this->objThread = std::thread(&Camera::objectDetection, this);
        this->objThread.detach();
    }

    void updateThresholds(std::string data) {
        // parse new data
    }

    void loadConfig(std::string path) {
        // load config from file

    }

    void objectDetection() {
        Mat frame, frame_HSV, frame_threshold;
        while (true) {
            cap >> frame;
            if (frame.empty()) {
                break;
            }

            // Convert from BGR to HSV colorspace
            cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

            // Detect the object based on HSV Range Values
            inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

            // Add filters erode & dilate
            erode(frame_threshold, frame_threshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
            dilate(frame_threshold, frame_threshold, getStructuringElement(cv::MORPH_ELLIPSE, kernel));

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
            if (contours.size()) {
                // Find the contour with the maximum area
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });

                // Calculate moments for the contour with maximum area
                Moments mu = moments(max_contour);
                int mu_area = mu.size();
                std::cout << "blob size: "
                
                //add 1e-5 to avoid division by zero
                Point2f mc = Point2f( static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
                std::static_cast<float>(mu.m01 / (mu.m00 + 1e-5)) );
                
                // Calculate the center of the image
                Point2f center(frame.cols / 2, frame.rows / 2);
                // Calculate distance. get negative distance if object is on right side. 
                double distance = mc.x > center.x ? - norm(mc - center) : norm(mc - center);
                cout << "Distance from center to contour : " << distance << endl;
            }
            // Wait for 100 ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};