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

typedef std::pair<std::string, std::vector<int>> Thresholds;
typedef std::pair<std::string, std::vector<Thresholds>> ColorsThresholds;


// Sets
extern std::vector<std::string> sets;
// Initialize the values for the trackbars for three sets
extern int all_values[3][6];
extern int set_index; // Global variable to keep track of the current set


/*
    * Thresholds for the object detection
    * The thresholds are in the following order:
    *  - Low H
    *  - High H
    *  - Low S
    *  - High S
    *  - Low V
    *  - High V
*/
// extern std::vector<std::vector<int>> Thresholds;

#define BLUE_THRESHOLD "BLUE"
#define PURPLE_THRESHOLD "PURPLE"
#define YELLOW_THRESHOLD "YELLOW"
// #define BLUE_THRESHOLD 0
// #define PURPLE_THRESHOLD 1
// #define YELLOW_THRESHOLD 2

class Camera {
    private:
        /* Object detection */
        int low_H = 87, low_S = 100, low_V = 43;
        int high_H = 113, high_S = 255, high_V = 255;
        const int max_value_H = 1000 / 2;
        const int max_value = 255;
        cv::Size kernel = cv::Size(5, 5);
        // int dilate_size = 12;
        // int max_dilate_size = 40;
        int dilate_size = 9;
        int max_dilate_size = 40;
        cv::VideoCapture cap;

        bool reverse = false;
        std::string lastProfile;

        int blocksGrabbed = 0;
        std::string currentThreshold = "BLUE";
        std::vector<ColorsThresholds> thresholds;
    

        std::vector<int> obstacles = {0, 0, 0};
        
        Mat frame, frame_threshold;
        cv::Point2f center;
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat drawing;
        std::vector<cv::Vec4i> hierarchy;

        /* Config */
        int inotifyFd;
        int watchFd;
        std::string filePath = "./Thresholds.conf";
        bool windowFlag = true;
        


        std::thread objThread;
        std::thread captureThread;
        std::mutex mtx;
        std::mutex capMtx;

        double distance = 0;
        double obstacleDistance = 0;
        double worldDistance = 100000; // was zero 
        bool blockGrabbed = false;

        std::string currentMode = "idle";
        std::vector<double> coordinates = {0, 0, 0};
    
    public:
        Camera();

        void setWindowFlag(bool flag);

        void start();

        double getDistance();

        double getWorldDistance();

        bool grabbed();

        void setGrabbed(bool flag);

        void getData();

        void handleBlue(std::vector<std::vector<cv::Point>> localContours);

        void handleObstacles(std::vector<std::vector<cv::Point>> localContours);
        
        void objectDetection();

        void printMode(std::string mode);

        void printCoordinates(double x, double y, double z);

        double getObstacleDistance();

        int getBlocksGrabbed();

        void setBlocksGrabbed(int blocks);

        // void getThreshold(int index);

        void readFile(const std::string &filePath);

        void setThresholds(int low_H, int low_S, int low_V, int high_H, int high_S, int high_V);

        std::vector<int> getThreshold(const std::string &color, const std::string &thresholdName);
        
        void setColorProfile(std::string color);

        bool shouldReverse();

        void setReverse(bool flag);
};