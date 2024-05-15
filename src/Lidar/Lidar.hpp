#include "../include.hpp"
#include "../Robot/Arena.hpp"
#include "../Display.hpp"

class Lidar {
    private:
        Arena *arena;
        int alpha = 0;
        int beta = 0;
        //int _gamma = (-90 * (int)M_PI / 180) / 4;
        int _gamma = (-90 * (int)M_PI / 180);

        double posX = 13.5;
        double posY = 10.5;
        double posA = 90*M_PI/180;//0;
        double ddx = 0;
        double ddy = 0;
        double dda = 0;
        Eigen::MatrixXd RMatrix = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(3, 3);
        // Eigen::MatrixXd positionMatrix = Eigen::MatrixXd::Zero(3, 1);
        Eigen::MatrixXd positionMatrixPoll = Eigen::MatrixXd::Zero(360, 3);
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

    public:
        Lidar(Arena arena, Display *disp, std::string path = "");

        ~Lidar();

        int isDataReady();

        Eigen::MatrixXd getData();

        void readFileData();

        void pollLidarData();

        float calculate_median(std::vector<double> list);

        bool cox_linefit();

        double getPosX();

        double getPosY();

        double getPosA();

        Eigen::MatrixXd getCovariance();

        void waitForUpdate();

        void updatePosition(double x, double y, double a);
};