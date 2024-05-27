#include "../pch.h"
#include "../Robot/Arena.hpp"
#include "../Display.hpp"

class Lidar {
    private:
        Arena *arena;
        int alpha = 0;
        int beta = 0;
        // int _gamma = (-180) * M_PI / 180;
        // int _gamma = (-90 * (int)M_PI / 180) / 4;
        double _gamma = ((-90) * M_PI / 180); // was -110 deg

        double posX = 13.5;
        double posY = 10.5;
        double posA = 90*M_PI/180;//0;
        double ddx = 0;
        double ddy = 0;
        double dda = 0;
        Eigen::MatrixXd RMatrix = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(3, 3);
        // Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(3, 1);
        Eigen::MatrixXd positionMatrixPoll = Eigen::MatrixXd::Zero(500, 3);
        Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(1, 3);
        Eigen::Matrix<double, 4, 4> line_model;
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(3, 3);
        int lines = 0;

        std::mutex mtx;

        bool _isPolling = false;
        ssize_t valread;
        char buffer2[1024] = {0};
        int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in serverAddress;
        int newSock;

        std::ifstream data;

        std::thread pollingThread;
        std::mutex positionMutex;
        int _positionsUpdated = 0;

        Display *display;

        std::ofstream cox;

    public:
        Lidar(Arena arena, Display *disp, std::string path = "");

        ~Lidar();

        int isDataReady();

        Eigen::MatrixXd getData();

        void readFileData();

        void pollLidarData();

        float normalize_angle(float angle);

        float calculate_median(std::vector<double> list);

        bool cox_linefit();

        double getPosX();

        double getPosY();

        double getPosA();

        Eigen::MatrixXd getCovariance();

        void waitForUpdate();

        void updatePosition(double x, double y, double a);

        void debug(double a1, double a2) {
            this->_gamma = a1 * M_PI / 180;
            this->posA = a2 * M_PI / 180;
        }

        void setAngle(double a) {
            this->posA = a;
        }
};