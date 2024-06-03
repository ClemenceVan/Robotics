#include "./Camera.hpp"

using namespace cv;


// std::vector<std::vector<int>> Thresholds = {
//     { // Blue
//         87, 113, 100, 255, 43, 255
//     },
//     { // Purple
//         118, 141, 79, 255, 0, 80
//     },
//     { // Yellow
//         25, 180, 105, 204, 82, 129
//     }
// };


// Sets
std::vector<std::string> sets = {"BLUE", "YELLOW", "PURPLE"};
// Initialize the values for the trackbars for three sets
int all_values[3][6] = {
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0}
};
int set_index = 0; // Global variable to keep track of the current set



// Function to get the index of an element 
int getIndex(std::vector<std::string> v, std::string K) { 
    auto it = find(v.begin(), v.end(), K);
    if (it != v.end()) return it - v.begin();
    else return -1;
}

void switch_set() {
    // Rotate between the sets
    set_index = (set_index + 1) % 3;
    std::cout << "Current Set: " << sets[set_index] << std::endl;

    // Retrieve the values for the current set
    // int(*all_values)[6] = (int(*)[6])all_values;
    int* values = all_values[set_index];

    std::cout << "values: " << values[0] << " " << values[1] << " " << values[2] << " " << values[3] << " " << values[4] << " " << values[5] << std::endl;


    // Update the trackbars with the values of the current set
    cv::setTrackbarPos("Hue low", "Trackbars", values[0]);
    cv::setTrackbarPos("Hue high", "Trackbars", values[1]);
    cv::setTrackbarPos("Sat low", "Trackbars", values[2]);
    cv::setTrackbarPos("Sat high", "Trackbars", values[3]);
    cv::setTrackbarPos("Val low", "Trackbars", values[4]);
    cv::setTrackbarPos("Val high", "Trackbars", values[5]);

    std::cout << "values: " << values[0] << " " << values[1] << " " << values[2] << " " << values[3] << " " << values[4] << " " << values[5] << std::endl;
}

void writeFile(std::string filePath) {
    
    all_values[set_index][0] = cv::getTrackbarPos("Hue low", "Trackbars");
    all_values[set_index][1] = cv::getTrackbarPos("Hue high", "Trackbars");
    all_values[set_index][2] = cv::getTrackbarPos("Sat low", "Trackbars");
    all_values[set_index][3] = cv::getTrackbarPos("Sat high", "Trackbars");
    all_values[set_index][4] = cv::getTrackbarPos("Val low", "Trackbars");
    all_values[set_index][5] = cv::getTrackbarPos("Val high", "Trackbars");
    std::ofstream file;
    file.open(filePath, std::ofstream::out | std::ofstream::trunc);
    
    for (int i = 0; i < 3; i++) {
        file << sets[i] << std::endl;
        file << "hue " << all_values[i][0] << " " << all_values[i][1] << std::endl;
        file << "saturation " << all_values[i][2] << " " << all_values[i][3] << std::endl;
        file << "value " << all_values[i][4] << " " << all_values[i][5] << std::endl;
        file << std::endl;
    }
    file.close();
}
// Callback function for trackbars
void on_trackbar(int, void* userdata) {
    // if (set_index == -1) return;
    // // int(*all_values)[6] = (int(*)[6])userdata;
    // // int* values = all_values[set_index];

    // all_values[set_index][0] = cv::getTrackbarPos("Hue low", "Trackbars");
    // all_values[set_index][1] = cv::getTrackbarPos("Hue high", "Trackbars");
    // all_values[set_index][2] = cv::getTrackbarPos("Sat low", "Trackbars");
    // all_values[set_index][3] = cv::getTrackbarPos("Sat high", "Trackbars");
    // all_values[set_index][4] = cv::getTrackbarPos("Val low", "Trackbars");
    // all_values[set_index][5] = cv::getTrackbarPos("Val high", "Trackbars");

    // std::cout << "Current Set: " << set_index << std::endl;
    // std::cout << "Hue Low: " << all_values[set_index][0] << " Hue High: " << all_values[set_index][1] << std::endl;
    // std::cout << "Saturation Low: " << all_values[set_index][2] << " Saturation High: " << all_values[set_index][3] << std::endl;
    // std::cout << "Value Low: " << all_values[set_index][4] << " Value High: " << all_values[set_index][5] << std::endl;
    // writeFile();
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

        if (convexityRatio > 0.9)
            convex++;
    }
    return !convex;
}

Camera::Camera() {
    cap.open(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Couldn't open the capture device." << std::endl;
        exit(1);
    }
    kernel = cv::Size(dilate_size, dilate_size);
    readFile(this->filePath);
    for (auto &set : sets) {
        int idx = getIndex(sets, set);
        std::cout << "set: " << set << " index: " << idx << std::endl;
        std::cout << "values: " << all_values[idx][0] << " " << all_values[idx][1] << " " << all_values[idx][2] << " " << all_values[idx][3] << " " << all_values[idx][4] << " " << all_values[idx][5] << std::endl;
    }
    // CameraConfig Config;
    // Config.loadConfig();
}

void Camera::setWindowFlag(bool flag) {
    this->windowFlag = flag;
    std::cout << "Window Flag: " << this->windowFlag << std::endl;
}

void Camera::start() {
    cap >> frame;
    // std::cout << "windowFlag: " << windowFlag << std::endl;
    if (frame.empty()) {
        std::cout << "Error: Couldn't capture frame." << std::endl;
        //break;
    }
    // Calculate the center of the image
    this->center = cv::Point(frame.cols / 2, frame.rows / 2);
    this->captureThread = std::thread(&Camera::getData, this);
    this->captureThread.detach();
    this->objThread = std::thread(&Camera::objectDetection, this);
    this->objThread.detach();
}

double Camera::getDistance() {
    mtx.lock();
    double _distance = this->distance;
    mtx.unlock();
    return _distance;
}

double Camera::getWorldDistance() {
    mtx.lock();
    double _distance = this->worldDistance;
    mtx.unlock();
    return _distance;
}

bool Camera::grabbed() {
    this->mtx.lock();
    bool _grabbed = this->blockGrabbed;
    this->mtx.unlock();
    return _grabbed;
}

void Camera::setGrabbed(bool flag) {
    this->mtx.lock();
    this->blockGrabbed = flag;
    this->mtx.unlock();
}

void Camera::getData() {
    Mat frame_HSV;
    bool flag = false;
    while (true) {
        if (!flag)
            capMtx.lock();
        // Capture the frame
        cap >> frame;

        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

        // Detect the object based on HSV Range Values
        // here call get Thresholds
        // std::vector<int> th = this->getThreshold(this->currentThreshold);
        this->lastProfile = this->currentThreshold;
        this->setColorProfile(this->lastProfile);
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), this->frame_threshold);

        // Add filters erode & dilate
        erode(this->frame_threshold, this->frame_threshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        dilate(this->frame_threshold, this->frame_threshold, getStructuringElement(cv::MORPH_ELLIPSE, kernel));

        cv::findContours(this->frame_threshold, this->contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if (this->lastProfile.compare(this->currentThreshold) != 0) { // Threshold changed during capture, can't serve data
            flag = true;
            continue;
        }
        flag = false;
        capMtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Camera::handleBlue(std::vector<std::vector<cv::Point>> localContours) {
    std::vector<cv::Point> max_contour;
    bool block = 0;
    cv::Rect rect;
    int one = 0;
    this->distance = 0;
    while ((this->distance == 0 || this->obstacles[set_index] == 0) && localContours.size()) {
        if (localContours.size() == 0)
            break;
        /* Pop contour with the biggest area */
        max_contour = *std::max_element(localContours.begin(), localContours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            }
        );
        localContours.erase(std::remove(localContours.begin(), localContours.end(), max_contour), localContours.end());

        /* remove all localContours outside of the bounding box of the max contour */
        std::vector<std::vector<cv::Point>> Block;
        cv::Rect bigRect = cv::boundingRect(max_contour);

        for (int i = 0; i < localContours.size(); i++) {
            cv::Rect rect2 = cv::boundingRect(localContours[i]);
            if (rect2.x >= bigRect.x &&
                rect2.y >= bigRect.y &&
                rect2.x + rect2.width <= bigRect.x + bigRect.width &&
                rect2.y + rect2.height <= bigRect.y + bigRect.height) {
                Block.push_back(localContours[i]);
            } else
                continue;
        }

        block = readNumber(max_contour, Block);

        if (this->obstacles[set_index] == 0 && block == 0) {
            double maxContourCenterX = bigRect.x + bigRect.width / 2;
            this->obstacles[set_index] = maxContourCenterX > center.x ? - norm(frame.cols - maxContourCenterX - bigRect.width) : norm(maxContourCenterX + rect.width); // give distance from edge of the frame to the center of the block
            // if (bigRect.y <= 10)
            //     this->reverse = true;
        }
        if (this->distance == 0 && block == 1) {
            rect = cv::boundingRect(max_contour);
            cv::Moments mu = cv::moments(max_contour);
            cv::Point2f mc = cv::Point2f(mu.m10 / (mu.m00 + 1e-5),
                                        mu.m01 / (mu.m00 + 1e-5));
            this->distance = mc.x > center.x ? norm(mc - center) : - norm(mc - center);
        }
        for (int i = 0; i < Block.size(); i++)
            localContours.erase(std::remove(localContours.begin(), localContours.end(), Block[i]), localContours.end());
    }

    if (this->distance == 0 && localContours.size() == 0)
        return;

    // // Calculate moments for the contour with maximum area
    // Moments mu = moments(max_contour);
    
    // //add 1e-5 to avoid division by zero
    // cv::Point2f mc = cv::Point2f(mu.m10 / (mu.m00 + 1e-5),
    //                             mu.m01 / (mu.m00 + 1e-5));

    //distance in real world
    double realDistance = (FOCAL_LENGTH * OBJ_WIDTH) / (rect.width);

    
    // Calculate distance between image center and obj center.
    // get negative distance if object is on right side.
    mtx.lock();
    // if (rect.y == 0 && rect.width > 250) {
    if (rect.y == 0 /*&& rect.width > 50*/) {
        this->blockGrabbed = true;
        // this->distance = 0;
    }
    this->worldDistance = realDistance;
    mtx.unlock();

    if (windowFlag) {
        // circle(drawing, mc, 4, Scalar(0, 0, 255), -1, 8, 0 );
        if (this->distance != 0) {
            putText(drawing, "width: " + std::to_string(rect.width) + " height: " + std::to_string(rect.height), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            putText(drawing, "distance: " + std::to_string(this->distance), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            putText(drawing, "real distance: " + std::to_string(realDistance), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            rectangle(drawing, rect, cv::Scalar(0, 0, 255), 2, 8, 0);
        }
    }
}

void Camera::handleObstacles(std::vector<std::vector<cv::Point>> localContours) {
    std::vector<cv::Point> max_contour;
    cv::Rect rect;

    while (this->obstacles[set_index] == 0 && localContours.size()) {
        if (localContours.size() == 0)
            return;
        /* Pop contour with the biggest area */
        max_contour = *std::max_element(localContours.begin(), localContours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            }
        );
        localContours.erase(std::remove(localContours.begin(), localContours.end(), max_contour), localContours.end());

        /* Remove all localContours outside of the bounding box of the max contour */
        std::vector<std::vector<cv::Point>> Block;
        rect = cv::boundingRect(max_contour);

        if (this->blocksGrabbed > 0 && rect.y == 0) // add backing up logic here maybe ? not sure if needed
            continue;

        for (int i = 0; i < localContours.size(); i++) {
            cv::Rect rect2 = cv::boundingRect(localContours[i]);
            if (rect2.x >= rect.x &&
                rect2.y >= rect.y &&
                rect2.x + rect2.width <= rect.x + rect.width &&
                rect2.y + rect2.height <= rect.y + rect.height) {
                Block.push_back(localContours[i]);
            } else
                continue;
        }
        
        double maxContourCenterX = rect.x + rect.width / 2;
        this->obstacles[set_index] = maxContourCenterX > center.x ? - norm(frame.cols - maxContourCenterX - rect.width) : norm(maxContourCenterX + rect.width); // give distance from edge of the frame to the center of the block
        std::cout << "obstacle distance: " << this->obstacles[set_index] << std::endl;
        std::cout << "SHOULD REVERSE? rect.y = "<<rect.y << std::endl;
        if (rect.y == 0)
            this->reverse = true;
        
        for (int i = 0; i < Block.size(); i++)
            localContours.erase(std::remove(localContours.begin(), localContours.end(), Block[i]), localContours.end());
    }
}

void Camera::objectDetection() {
    if (windowFlag) {
        cv::namedWindow("Frame");
        cv::namedWindow("Contours");
        cv::namedWindow("Trackbars", cv::WINDOW_AUTOSIZE);
    }

    int(*tmpvals)[6] = (int(*)[6])all_values;

     // Create trackbars and attach them to the window
    cv::createTrackbar("Hue high", "Trackbars", NULL, 255, on_trackbar);
    cv::createTrackbar("Hue low", "Trackbars", NULL, 255, on_trackbar);
    cv::createTrackbar("Sat low", "Trackbars", NULL, 255, on_trackbar);
    cv::createTrackbar("Sat high", "Trackbars", NULL, 255, on_trackbar);
    cv::createTrackbar("Val low", "Trackbars", NULL, 255, on_trackbar);
    cv::createTrackbar("Val high", "Trackbars", NULL, 255, on_trackbar);
    
    cv::setTrackbarPos("Hue high", "Trackbars", all_values[0][1]);
    cv::setTrackbarPos("Hue low", "Trackbars", all_values[0][0]);
    cv::setTrackbarPos("Sat low", "Trackbars", all_values[0][2]);
    cv::setTrackbarPos("Sat high", "Trackbars", all_values[0][3]);
    cv::setTrackbarPos("Val low", "Trackbars", all_values[0][4]);
    cv::setTrackbarPos("Val high", "Trackbars", all_values[0][5]);

    bool forced_flag = false;

    
    /* Wait for the first frame to be captured */
    while(capMtx.try_lock()) {
        capMtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    int counter = 0;
    while (true) {
        /* Initialize variables */
        this->obstacles[set_index] = 0;
        drawing = cv::Mat::zeros(frame_threshold.size(), CV_8UC3);
        this->capMtx.lock();
        if (this->lastProfile.compare(this->currentThreshold) != 0) {
            this->capMtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            continue;
        }
        std::vector<std::vector<cv::Point>> localContours = this->contours;
        this->capMtx.unlock();

        /* Draw contours */
        if (windowFlag)
            for (size_t i = 0; i < localContours.size(); i++)
                cv::drawContours(drawing, localContours, (int)i, cv::Scalar(0, 255, 0), 2, cv::LINE_8, hierarchy, 0);

        if (localContours.size()) {
            if (this->currentThreshold == BLUE_THRESHOLD)
                this->handleBlue(localContours);
            else
                this->handleObstacles(localContours);
        } else {
            mtx.lock();
            std::cout << "NO BLOCKS" << std::endl;
            if (this->currentThreshold == BLUE_THRESHOLD) {
                /**
                 * fix no obstacle avoidance on going home
                 * going back and forth while reversing 
                 * 
                 */
                this->distance = 0;
                this->worldDistance = 0;
            }
            mtx.unlock();
        }

        if (windowFlag) {
            putText(drawing, "mode: " + this->currentMode, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            putText(drawing, "x: " + std::to_string((int)this->coordinates[0]) + " y: " + std::to_string((int)this->coordinates[1]) + " a: " + std::to_string(this->coordinates[2]), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            putText(drawing, "obstacle distance: " + std::to_string(this->getObstacleDistance()), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            putText(drawing, "blocks grabbed: " + std::to_string(this->blocksGrabbed), cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            putText(drawing, "threshold: " + this->currentThreshold, cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
            // Draw the contour and center of the shape on the image
            // cv::drawContours(frame, localContours, -1, cv::Scalar(0, 255, 0), 2);
            // cv::circle(frame, mc, 4, cv::Scalar(0, 0, 255), -1);
            cv::imshow("Frame", frame);
            cv::imshow("Contours", drawing);

            // Create an empty image to display the values
            cv::Mat display_image(300, 512, CV_8UC3, cv::Scalar(0, 0, 0));

            // Display the values of the current set
            cv::putText(display_image, "Current Set: " + sets[set_index], cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::putText(display_image, "Space to switch profile, enter to save", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::putText(display_image, "A to force profile on robot : " + std::to_string(forced_flag), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::putText(display_image, "Obstacle Distance: " + std::to_string(this->obstacles[set_index]), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            // Show the image
            cv::imshow("Trackbars", display_image);

            char key = (char)waitKey(30);
            if (key == 32) // Switch set if the user presses the spacebar            
                switch_set();
            else if (key == 13) { // Save the values if the user presses the enter key
                writeFile(this->filePath);
                readFile(this->filePath);
            } else if (key == 'a')
                forced_flag = !forced_flag;
        }
        if (!forced_flag) {
            counter++;
            if (counter == 5)
                this->currentThreshold = PURPLE_THRESHOLD;
            else if (counter == 6)
                this->currentThreshold = YELLOW_THRESHOLD;
            else if (counter == 7) {
                this->currentThreshold = BLUE_THRESHOLD;
                counter = 0;
            }
            set_index = getIndex(sets, this->currentThreshold);
        } else {
            this->currentThreshold = sets[set_index];
        }

    }
}

void Camera::printMode(std::string mode) {
    this->currentMode = mode;
}

void Camera::printCoordinates(double x, double y, double z) {
    this->coordinates = {x, y, z};
}

double Camera::getObstacleDistance() {
    mtx.lock();
    auto tmp = std::max_element(this->obstacles.begin(), this->obstacles.end(), [](int a, int b) { return std::abs(a) < std::abs(b); });
    if (tmp != this->obstacles.end())
        this->obstacleDistance = *tmp;
    else
        this->obstacleDistance = 0;
    double _distance = this->obstacleDistance;
    // double _distance = 0;
    mtx.unlock();
    return _distance;
}

int Camera::getBlocksGrabbed() {
    return this->blocksGrabbed;
}

void Camera::setBlocksGrabbed(int blocks) {
    this->blocksGrabbed = blocks;
}

// void Camera::getThreshold(int index)  {
//     if (index < 0 || index >= Thresholds.size())
//         index = 0;
//     this->low_H = Thresholds[index][0];
//     this->high_H = Thresholds[index][1];
//     this->low_S = Thresholds[index][2];
//     this->high_S = Thresholds[index][3];
//     this->low_V = Thresholds[index][4];
//     this->high_V = Thresholds[index][5];
// }

void Camera::readFile(const std::string &filePath) {
    std::ifstream file(this->filePath);
    if (file.is_open()) {
        std::string line;
        this->thresholds.clear(); // Clear existing thresholds before reading new ones
        while (getline(file, line)) {
            // Skip or comments
            if (/*line.empty() || */line[0] == '#') continue;

            std::string color;
            std::vector<Thresholds> colorThresholds;

            // Read the color name
            color = line;

            // Read the thresholds for the color
            while (getline(file, line)) {
                if (line.empty() || line.size() == 1) break; // Break on empty line

                std::istringstream iss(line);
                std::string thresholdName;
                std::vector<int> thresholdValues;
                iss >> thresholdName;
                int value;
                while (iss >> value) {
                    thresholdValues.push_back(value);
                }
                colorThresholds.push_back(std::make_pair(thresholdName, thresholdValues));
            }
            int idx = getIndex(sets, color);
            all_values[idx][0] = colorThresholds[0].second[0];
            all_values[idx][1] = colorThresholds[0].second[1];
            all_values[idx][2] = colorThresholds[1].second[0];
            all_values[idx][3] = colorThresholds[1].second[1];
            all_values[idx][4] = colorThresholds[2].second[0];
            all_values[idx][5] = colorThresholds[2].second[1];

            // Add the color and its thresholds to the overall thresholds
            this->thresholds.push_back(std::make_pair(color, colorThresholds));
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << this->filePath << std::endl;
        throw std::runtime_error("Unable to open file: " + this->filePath);
    }
}

void Camera::setColorProfile(std::string color) {
    // set thresholds based on color from config file
    std::vector<int> hue = this->getThreshold(color, "hue");
    std::vector<int> saturation = this->getThreshold(color, "saturation");
    std::vector<int> value = this->getThreshold(color, "value");
    if (!hue.empty() && !saturation.empty() && !value.empty()) {
        setThresholds(hue[0], saturation[0], value[0], hue[1], saturation[1], value[1]);
    } else {
        std::cerr << "Error: Couldn't find thresholds for color " << color << std::endl;
        throw std::runtime_error("Error: Couldn't find thresholds for color " + color);
    }
}

void Camera::setThresholds(int low_H, int low_S, int low_V, int high_H, int high_S, int high_V) {
    this->low_H = low_H;
    this->low_S = low_S;
    this->low_V = low_V;
    this->high_H = high_H;
    this->high_S = high_S;
    this->high_V = high_V;
}

std::vector<int> Camera::getThreshold(const std::string &color, const std::string &thresholdName) {
    for (const auto &colorThresholds : thresholds) {
        // std::cout << "test " <<  colorThresholds.first << " color " << color << std::endl;
        if (colorThresholds.first == color || colorThresholds.first == color + "\r") {
            for (const auto &threshold : colorThresholds.second) {
                if (threshold.first == thresholdName) {
                    return threshold.second;
                }
            }
        }
    }
    return {};
    // throw std::runtime_error("Threshold not found! Edit the config file to add the threshold.");
}

bool Camera::shouldReverse() {
    return this->reverse;
}

void Camera::setReverse(bool flag) {
    this->reverse = flag;
}