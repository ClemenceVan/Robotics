#include "../include.hpp"
#include "../Display.hpp"
#include "../Arena.hpp"
// #include "asio/include/asio.hpp"
// #include <asio.hpp>



class Lidar {
    private:
    Arena *arena;
    int alpha = 0;
    int beta = 0;
    int _gamma = (-90 * (int)M_PI / 180) / 4;
    double posX = 13.5; // to be changed when start
    double posY = 10.5; // to be changed when start
    int start_angle = 0;
    double ddx = 0;
    double ddy = 0;
    double dda = 0;
    Eigen::MatrixXd RMatrix = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(3, 3);
    // Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd positionMatrixPoll = Eigen::MatrixXd::Zero(360, 3);
    Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(1, 3);
    Eigen::Matrix<double, 4, 4> line_model;
    int lines = 0;

    std::mutex mtx;

    Display *display;

    bool _isPolling = false;
    ssize_t valread;
    char buffer2[1024] = {0};
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    int newSock;

    std::ifstream data;

    std::thread pollingThread;

    public:
    Lidar(Arena arena, bool display = true, std::string path = "");

    ~Lidar();

    int isDataReady() {
        mtx.lock();
        int l = this->lines;
        mtx.unlock();
        if (l >= 359) return 1;
        return 0;
    }

    Eigen::MatrixXd getData() {
        mtx.lock();
        Eigen::MatrixXd m = this->positionMatrixPoll;
        mtx.unlock();
        return m;
    }

    void readFileData();

    void pollLidarData();

    float calculate_median(std::vector<double> list);

    void cox_linefit();
};