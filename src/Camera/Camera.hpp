#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/inotify.h>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>
#include <algorithm>
#include <math.h>

using namespace cv;

#define FOCAL_LENGTH 639 // Focal length of the camera in pixels
#define OBJ_WIDTH 8 // Width of the object in cm

class Camera { 
    private:
    /* Object detection */
    int low_H = 87, low_S = 100, low_V = 43;
    int high_H = 113, high_S = 255, high_V = 255;
    const int max_value_H = 1000 / 2;
    const int max_value = 255;
    cv::Size kernel = cv::Size(5, 5);
    int dilate_size = 12;
    int max_dilate_size = 40;
    cv::VideoCapture cap;

    int blocksGrabbed = 0;

    /* Config */
    int inotifyFd;
    int watchFd;
    std::string filePath = "./config/thresholds.conf";
    bool windowFlag = true;

    std::thread objThread;
    std::mutex mtx;

    double distance = 0;
    double obstacleDistance = 0;
    double worldDistance = 100000; // was zero 
    bool blockGrabbed = false;

    std::string currentMode = "idle";
    std::vector<double> coordinates = {0, 0, 0};
    public:
    Camera() {
        std::cout << "Camera object created" << std::endl;
        cap.open(0);
        if (!cap.isOpened()) {
            std::cerr << "Error: Couldn't open the capture device." << std::endl;
            exit(1);
        }
        std::cout << "Camera opened" << std::endl;
        kernel = cv::Size(dilate_size, dilate_size);
        // CameraConfig Config;
        // Config.loadConfig();
    }

    void setWindowFlag(bool flag) {
        this->windowFlag = flag;
        std::cout << "Window Flag: " << this->windowFlag << std::endl;
    }

    void start() {
        std::cout << "Starting camera thread" << std::endl;
        this->objThread = std::thread(&Camera::objectDetection, this);
        std::cout << "Camera thread started" << std::endl;
        this->objThread.detach();
        std::cout << "Camera thread detached" << std::endl;
    }

    void updateThresholds(std::string data) {
        // parse new data
    }

    void loadConfig(std::string path) {
        // load config from file

    }

    double getDistance() {
        mtx.lock();
        double _distance = this->distance;
        // double _distance = 0;
        mtx.unlock();
        return _distance;
    }

    double getWorldDistance() {
        mtx.lock();
        double _distance = this->worldDistance;
        mtx.unlock();
        return _distance;
    }

    bool grabbed() {
        this->mtx.lock();
        bool _grabbed = this->blockGrabbed;
        this->mtx.unlock();
        return _grabbed;
    }

    bool readNumber(std::vector<cv::Point> contour, std::vector<std::vector<cv::Point>> contours) {

        /* Check if any of the contours are convex */
        int convex = 0;
        for (int i = 0; i < contours.size(); i++) {
            // Calculate the area of the contour
            double contourAreaValue = cv::contourArea(contours[i]);

            // Find the convex hull of the contour
            std::vector<cv::Point> hull;
            cv::convexHull(contours[i], hull);

            // Calculate the area of the convex hull
            double hullAreaValue = cv::contourArea(hull);

            // Calculate the convexity ratio
            double convexityRatio = contourAreaValue / hullAreaValue;

            // Print the results
            // std::cout << "Contour #" << i << " Convexity Ratio: " << convexityRatio << std::endl;

            if (convexityRatio > 0.9)
                convex++;
        }
        // std::cout << "convex: " << convex << std::endl;
        // if(convex > 0)
        //     return 0;
        // else
        //     return 1;
        return !convex;
    }

    void objectDetection() {
        std::cout << "Object detection thread function" << std::endl;
        Mat frame, frame_HSV, frame_threshold;
        cv::Mat drawing;
        if (windowFlag) {
            cv::namedWindow("Frame");
            cv::namedWindow("Contours");
        }
        while (true) {
            this->obstacleDistance = 0;
            // std::cout << "Object detection loop" << std::endl;
            cap >> frame;
            // Calculate the center of the image
            cv::Point2f center(frame.cols / 2, frame.rows / 2);
            // std::cout << "windowFlag: " << windowFlag << std::endl;
            if (frame.empty()) {
                std::cout << "Error: Couldn't capture frame." << std::endl;
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

            if (windowFlag) {       
                // Draw contours
                drawing = cv::Mat::zeros(frame_threshold.size(), CV_8UC3);
                cv::Scalar color = cv::Scalar(0, 255, 0);
                for (size_t i = 0; i < contours.size(); i++) {
                    cv::drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
                }
            }

            if (contours.size()) {
                std::vector<cv::Point> max_contour;
                bool block = 0;
                cv::Rect rect;
                int one = 0;
                while (block != 1 && contours.size()) {
                    if (contours.size() == 0)
                        break;
                    /* Pop contour with the biggest area */
                    max_contour = *std::max_element(contours.begin(), contours.end(),
                        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                            return cv::contourArea(a) < cv::contourArea(b);
                        }
                    );
                    contours.erase(std::remove(contours.begin(), contours.end(), max_contour), contours.end());


                    /* remove all contours outside of the bounding box of the max contour */
                    std::vector<std::vector<cv::Point>> Block;
                    rect = cv::boundingRect(max_contour);

                    if (this->blocksGrabbed > 0 && rect.y == 0)
                        continue;

                    for (int i = 0; i < contours.size(); i++) {
                        // if (contours[i].size() < 10) {
                        //     // Block.push_back(contours[i]);
                        //     continue;
                        // }
                        cv::Rect rect2 = cv::boundingRect(contours[i]);
                        if (rect2.x >= rect.x &&
                            rect2.y >= rect.y &&
                            rect2.x + rect2.width <= rect.x + rect.width &&
                            rect2.y + rect2.height <= rect.y + rect.height) {
                            Block.push_back(contours[i]);
                        } else
                            continue;
                    }

                    block = this->readNumber(max_contour, Block);

                    if (block == 1) {
                        one++;
                        if (one > blocksGrabbed)
                            break;
                    }

                    // std::cout << "block: " << block << std::endl;
                    if (this->obstacleDistance == 0 && block == 0) {
                        double maxContourCenterX = rect.x + rect.width / 2;
                        this->obstacleDistance = maxContourCenterX > center.x ? - norm(frame.cols - maxContourCenterX - rect.width) : norm(maxContourCenterX + rect.width); // give distance from edge of the frame to the center of the block
                    }
                    for (int i = 0; i < Block.size(); i++)
                        contours.erase(std::remove(contours.begin(), contours.end(), Block[i]), contours.end());
                }
                if (block == 0 && contours.size() == 0) {
                    // std::cout << "NO BLOCKS" << std::endl;
                    // Wait for 100 ms
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                // std::cout << "VALID BLOCK FOUND" << std::endl;

                // std::cout<< "max contour area = " << contourArea(max_contour) << std::endl;
                // std::cout<< "frame area = " <<  frame.cols * frame.rows << std::endl;
                // if(contourArea(max_contour) >= frame.cols * frame.rows / 2.8 )
                //     std::cout << "GRABBED BLOCK" << std::endl;

                // Calculate moments for the contour with maximum area
                Moments mu = moments(max_contour);
                
                //add 1e-5 to avoid division by zero
                cv::Point2f mc = cv::Point2f(mu.m10 / (mu.m00 + 1e-5),
                                             mu.m01 / (mu.m00 + 1e-5));
                
                


                //distance in real world
                double realDistance = (FOCAL_LENGTH * OBJ_WIDTH) / (rect.width);

                
                // Calculate distance between image center and obj center.
                // get negative distance if object is on right side.
                mtx.lock();
                // std::cout<< "max contour area = " << contourArea(max_contour) << std::endl;
                // std::cout<< "frame area = " <<  frame.cols * frame.rows << std::endl;
                  // if (contourArea(max_contour) >= frame.cols * frame.rows / 2.8 && rect.y  < frame.height) {
                // std::cout << "rect.x = " << rect.x << ", rect.y = "<< rect.y << std::endl;
                // std::cout << "rect.width = " << rect.width << ", rect.height= "<< rect.height << std::endl;
                
                this->distance = mc.x > center.x ? norm(mc - center) : - norm(mc - center);
                if (rect.y == 0 && rect.width > 250) {
              
                    std::cout << "BLOCK GRABBED" << std::endl;
                    this->blockGrabbed = true;
                    this->distance = 0;
                } else {
                    this->blockGrabbed = false;
                }
                this->worldDistance = realDistance;
                mtx.unlock();

                if (windowFlag) {
                    circle( drawing, mc, 4, Scalar(0, 0, 255), -1, 8, 0 );
                    // rectangle(drawing, rect, cv::Scalar(0, 0, 255), 2, 8, 0);

                    if (block == 1) {
                        putText(drawing, "width: " + std::to_string(rect.width) + " height: " + std::to_string(rect.height), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                        putText(drawing, "distance: " + std::to_string(this->distance), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                        putText(drawing, "real distance: " + std::to_string(realDistance), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                        rectangle(drawing, rect, cv::Scalar(0, 0, 255), 2, 8, 0);
                    }
                    // for (int i = 0; i < contours.size(); i++) {
                    //     cv::Rect rect2 = cv::boundingRect(contours[i]);
                    //     rectangle(drawing, rect2, cv::Scalar(255, 0, 255), 2, 8, 0);
                    // }
                }
            } else {
                mtx.lock();
                std::cout << "NO BLOCKS" << std::endl;
                this->distance = 0;
                this->worldDistance = 0;
                mtx.unlock();
            }
            if (windowFlag) {
                putText(drawing, "mode: " + this->currentMode, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                putText(drawing, "x: " + std::to_string((int)this->coordinates[0]) + " y: " + std::to_string((int)this->coordinates[1]) + " a: " + std::to_string(this->coordinates[2]), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                putText(drawing, "obstacle distance: " + std::to_string(this->obstacleDistance), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                putText(drawing, "blocks grabbed: " + std::to_string(this->blocksGrabbed), cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                // Draw the contour and center of the shape on the image
                // cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2);
                // cv::circle(frame, mc, 4, cv::Scalar(0, 0, 255), -1);
                // cv::imshow("Frame", frame);
                cv::imshow("Contours", drawing);

                char key = (char)waitKey(30);
            }
            // Wait for 100 ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void printMode(std::string mode) {
        this->currentMode = mode;
    }

    void printCoordinates(double x, double y, double z) {
        this->coordinates = {x, y, z};
    }

    double getObstacleDistance() {
        mtx.lock();
        double _distance = this->obstacleDistance;
        // double _distance = 0;
        mtx.unlock();
        return _distance;
    }

    int getBlocksGrabbed() {
        return this->blocksGrabbed;
    }

    void setBlocksGrabbed(int blocks) {
        this->blocksGrabbed = blocks;
    }
};


               
